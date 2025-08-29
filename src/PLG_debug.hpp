
#if defined(DEBUG)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)     // Định nghĩa hàm in ra thông tin debug
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__) // Định nghĩa hàm in ra thông tin debug kèm xuống dòng
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)   // Định nghĩa hàm in ra thông tin debug theo định dạng
#else
// Nếu không ở chế độ DEBUG, định nghĩa các hàm này là rỗng
#define DEBUG_PRINT(...) // Nếu không ở chế độ DEBUG, không làm gì cả
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif
#if defined(SEN)
#define SEN_PRINT(...) Serial2.print(__VA_ARGS__)     // Định nghĩa hàm in ra thông tin debug
#define SEN_PRINTLN(...) Serial2.println(__VA_ARGS__) // Định nghĩa hàm in ra thông tin debug kèm xuống dòng
#define SEN_PRINTF(...) Serial2.printf(__VA_ARGS__)   // Định nghĩa hàm in ra thông tin debug theo định dạng
#else
// Nếu không ở chế độ DEBUG, định nghĩa các hàm này là rỗng
#define SEN_PRINT(...) // Nếu không ở chế độ DEBUG, không làm gì cả
#define SEN_PRINTLN(...)
#define SEN_PRINTF(...)
#endif