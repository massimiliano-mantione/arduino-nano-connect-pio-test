#[repr(u8)]
pub enum Vl53LxDistanceMode {
    Short = 1,
    Medium = 2,
    Long = 3,
}

#[repr(u8)]
pub enum Vl53LxOffsetCorrectionMode {
    Standard = 1,
    PerVcsel = 3,
}

#[repr(u8)]
pub enum Vl53LxSmudgeCorrectionMode {
    None = 0,
    Continuous = 1,
    Single = 2,
    Debug = 3,
}

#[repr(u8)]
pub enum Vl53LxPresetMode {
    None = 0,
    StandardRanging = 1,
    StandardRangingShortRange = 2,
    StandardRangingLongRange = 3,
    StandardRangingMm1Cal = 4,
    StandardRangingMm2Cal = 5,
    TimedRanging = 6,
    TimedRangingShortRange = 7,
    TimedRangingLongRange = 8,
    NearFarRanging = 9,
    QuadrantRanging = 10,
    HistogramRanging = 11,
    HistogramRangingShortTiming = 12,
    HistogramCharacterization = 13,
    HistogramXtalkPlanar = 14,
    HistogramXtalkMm1 = 15,
    HistogramXtalkMm2 = 16,
    Olt = 17,
    SingleshotRanging = 18,
    HistogramRefArray = 19,
    HistogramRangingWithMm1 = 20,
    HistogramRangingWithMm2 = 21,
    HistogramRangingMm1Cal = 22,
    HistogramRangingMm2Cal = 23,
    HistogramMultizone = 24,
}
