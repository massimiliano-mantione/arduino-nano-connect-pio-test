use btleplug::api::{
    BDAddr, Central, CharPropFlags, Characteristic, Manager as _, Peripheral, ScanFilter, WriteType,
};
use btleplug::platform::Manager;
use futures::stream::StreamExt;
use std::error::Error;
use std::str::FromStr;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

const PERIPHERAL_ADDRESS: &str = "84:cc:a8:78:35:de";
const TEST_SERVICE_UUID: Uuid = Uuid::from_u128(0x934642e9_db09_4802_90b2_aec03c8bd146);
const DATA_CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x2a47b36e_3e2e_496f_9a8e_2f14313acb53);
const COMMAND_CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x2d4b86f0_2342_4d67_8734_9c2ca4b380d6);
const DATA_CHARACTERISTIC: Characteristic = Characteristic {
    uuid: DATA_CHARACTERISTIC_UUID,
    service_uuid: TEST_SERVICE_UUID,
    properties: CharPropFlags::READ.union(CharPropFlags::NOTIFY),
};
const COMMAND_CHARACTERISTIC: Characteristic = Characteristic {
    uuid: COMMAND_CHARACTERISTIC_UUID,
    service_uuid: TEST_SERVICE_UUID,
    properties: CharPropFlags::READ.union(CharPropFlags::WRITE),
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let wanted_device: BDAddr = BDAddr::from_str(PERIPHERAL_ADDRESS).unwrap();

    let manager = Manager::new().await?;
    let adapter_list = manager.adapters().await?;
    if adapter_list.is_empty() {
        eprintln!("No Bluetooth adapters found");
    }

    for adapter in adapter_list.iter() {
        println!("Starting scan...");
        adapter
            .start_scan(ScanFilter::default())
            .await
            .expect("Can't scan BLE adapter for connected devices...");
        time::sleep(Duration::from_secs(2)).await;
        let peripherals = adapter.peripherals().await?;

        if peripherals.is_empty() {
            eprintln!("->>> BLE peripheral devices were not found, sorry. Exiting...");
        } else {
            // All peripheral devices in range.
            for peripheral in peripherals.iter().cloned() {
                let properties = peripheral.properties().await?.unwrap();
                let is_connected = peripheral.is_connected().await?;
                let local_name = properties
                    .local_name
                    .unwrap_or(String::from("(peripheral name unknown)"));
                //println!(
                //    "Peripheral {:?} is connected: {:?}",
                //    &local_name, is_connected
                //);
                // Check if it's the peripheral we want.
                if peripheral.address() == wanted_device {
                    println!("Found matching peripheral {:?}...", &local_name);
                    if !is_connected {
                        // Connect if we aren't already connected.
                        if let Err(err) = peripheral.connect().await {
                            eprintln!("Error connecting to peripheral, skipping: {}", err);
                            continue;
                        }
                    }
                    let is_connected = peripheral.is_connected().await?;
                    println!(
                        "Now connected ({:?}) to peripheral {:?}.",
                        is_connected, &local_name
                    );
                    if is_connected {
                        println!("Discover peripheral {:?} services...", local_name);
                        peripheral.discover_services().await?;
                        if peripheral.characteristics().contains(&DATA_CHARACTERISTIC) {
                            println!("Subscribing to characteristic {}", DATA_CHARACTERISTIC_UUID);
                            peripheral.subscribe(&DATA_CHARACTERISTIC).await?;
                            let mut notification_stream = peripheral.notifications().await?;
                            tokio::spawn(async move {
                                while let Some(data) = notification_stream.next().await {
                                    let text = String::from_utf8_lossy(&data.value);
                                    println!("EVENT: {}", text)
                                }
                            });
                        } else {
                            println!("ERROR: data characteristic not found");
                        }
                        if peripheral
                            .characteristics()
                            .contains(&COMMAND_CHARACTERISTIC)
                        {
                            let mut counter = 0;
                            loop {
                                tokio::time::sleep(Duration::from_millis(1000)).await;
                                counter += 1;
                                let mut cmd = format!("CMD:{}", counter);
                                while cmd.len() < 10 {
                                    cmd.push(' ');
                                }
                                println!("WRITING CMD: '{}'", &cmd);
                                peripheral
                                    .write(
                                        &COMMAND_CHARACTERISTIC,
                                        cmd.as_bytes(),
                                        WriteType::WithoutResponse,
                                    )
                                    .await
                                    .unwrap();
                            }
                        } else {
                            println!("ERROR: command characteristic not found");
                        }
                        //println!("Disconnecting from peripheral {:?}...", local_name);
                        //peripheral.disconnect().await?;
                    }
                }
            }
        }
    }
    Ok(())
}
