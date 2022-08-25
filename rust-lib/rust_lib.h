
extern "C" void print_tick(int tick);

typedef char i8;
typedef unsigned char u8;

typedef struct {
    i8 error_code;
    u8 address;
} Vl53LxData;
