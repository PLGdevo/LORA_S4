/* ---------------------------------------------------------------------
 * PLG define cac mode doc chuoi du lieu\--------------------------*/
#include <Arduino.h>
#include <PLG_debug.hpp>
#include <string.h>

String address = "";
String address_slave = ""; // Address of the slave device
String address_remus = "";
String namedata = "";
String data = "";
String data1 = "";
String data2 = "";
String data3 = "";
String data4 = "";
String receivedData = "";
String messages4 = address + "|" + address_slave + "|" + namedata + "|" + data;
String messages5 = address + "|" + address_slave + "|" + address_remus + "|" + namedata + "|" + data;
String messages_sensor = address + "|" + address_slave + "|" + namedata + "|" + data1 + "|" + data2 + "|" + data3 + "|" + data4;
/*                  dia chi gui | dia chi nhan | ten du lieu | du lieu
                    dc su dung cho 2 module                                 */
void PLG_write_4(String address, String address_slave, String namedata, String data)
{
    messages4 = address + "|" + address_slave + "|" + namedata + "|" + data;
    // DEBUG_PRINT("Sending packet: ");
    // DEBUG_PRINTLN(messages4);
}
void PLG_write_board_sensor(String address, String address_slave, String namedata, String data1, String data2, String data3, String data4)
{
    messages_sensor = address + "|" + address_slave + "|" + namedata + "|" + data1 + "|" + data2 + "|" + data3 + "|" + data4;
    // DEBUG_PRINT("Sending packet: ");
    // DEBUG_PRINTLN(messages_sensor);
}
/*         dia chi gui | dia chi trung gian |dia chi cuoi | ten du lieu | du lieu
                        su dung cho 3 module                                */
void PLG_write_5(String address, String address_slave, String address_remus, String namedata, String data)
{
    messages5 = address + "|" + address_slave + "|" + address_remus + "|" + namedata + "|" + data;
    // DEBUG_PRINT("Sending packet: ");
    // DEBUG_PRINTLN(messages5);
}
void PLG_read_4()
{
    int index1 = receivedData.indexOf('|');
    int index2 = receivedData.indexOf('|', index1 + 1);
    int index3 = receivedData.indexOf('|', index2 + 1);

    if (index1 == -1 || index2 == -1 || index3 == -1)
    {
        DEBUG_PRINTLN("Chuỗi không hợp lệ!");
        return;
    }

    address = receivedData.substring(0, index1);
    address_slave = receivedData.substring(index1 + 1, index2);
    namedata = receivedData.substring(index2 + 1, index3);
    data = receivedData.substring(index3 + 1);
    messages4 = address + "|" + address_slave + "|" + namedata + "|" + data;
    // DEBUG_PRINT("Sending packet: ");
    // DEBUG_PRINTLN(messages4);
}
void PLG_read_5()
{
    int i1 = receivedData.indexOf('|');
    int i2 = receivedData.indexOf('|', i1 + 1);
    int i3 = receivedData.indexOf('|', i2 + 1);
    int i4 = receivedData.indexOf('|', i3 + 1);

    if (i1 == -1 || i2 == -1 || i3 == -1 || i4 == -1)
    {
        Serial.println("Chuỗi không hợp lệ!");
        return;
    }
    address = receivedData.substring(0, i1);
    address_slave = receivedData.substring(i1 + 1, i2);
    address_remus = receivedData.substring(i2 + 1, i3);
    namedata = receivedData.substring(i3 + 1, i4);
    data = receivedData.substring(i4 + 1);
    messages5 = address + "|" + address_slave + "|" + address_remus + "|" + namedata + "|" + data;
    // DEBUG_PRINT("Sending packet: ");
    // DEBUG_PRINTLN(messages5);
}
void PLG_read_7()
{
    int i1 = receivedData.indexOf('|');
    int i2 = receivedData.indexOf('|', i1 + 1);
    int i3 = receivedData.indexOf('|', i2 + 1);
    int i4 = receivedData.indexOf('|', i3 + 1);
    int i5 = receivedData.indexOf('|', i4 + 1);
    int i6 = receivedData.indexOf('|', i5 + 1);

    if (i1 == -1 || i2 == -1 || i3 == -1 || i4 == -1 || i5 == -1 || i6 == -1)
    {
        Serial.println("Chuỗi không hợp lệ!");
        return;
    }

    address = receivedData.substring(0, i1);
    address_slave = receivedData.substring(i1 + 1, i2);
    namedata = receivedData.substring(i2 + 1, i3);
    data1 = receivedData.substring(i3 + 1, i4);
    data2 = receivedData.substring(i4 + 1, i5);
    data3 = receivedData.substring(i5 + 1, i6);
    data4 = receivedData.substring(i6 + 1);
    messages_sensor = address + "|" + address_slave + "|" + namedata + "|" + data1 + "|" + data2 + "|" + data3 + "|" + data4;
    // DEBUG_PRINTLN(messages_sensor);
}
// Hàm đếm số dấu '|' trong chuỗi
int PLG_count_separator(String str)
{
    int count = 0;
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) == '|')
        {
            count++;
        }
    }
    return count;
}

// Hàm kiểm tra chuỗi 4 hay 5 trường
void PLG_check_message()
{
    int soDauPhanCach = PLG_count_separator(receivedData);

    if (soDauPhanCach == 3)
    {
        // DEBUG_PRINTLN("Chuỗi có 4 trường dữ liệu.");
        PLG_read_4();
    }
    else if (soDauPhanCach == 4)
    {
        // DEBUG_PRINTLN("Chuỗi có 5 trường dữ liệu.");
        PLG_read_5();
    }
    else if (soDauPhanCach == 6)
    {
        // DEBUG_PRINTLN("Chuỗi có 7 trường dữ liệu.");
        PLG_read_7();
    }

    else
    {
        DEBUG_PRINT("Lỗi: Chuỗi không hợp lệ. Dấu '|': ");
        DEBUG_PRINTLN(soDauPhanCach);
    }
}

/* ban chi can goi ham nay ra la moi viec se dc giai quyet
if (Serial2.available())
    {
        receivedData = Serial2.readStringUntil('\n'); // Ví dụ nhận chuỗi
        PLG_check_message();                          // Gọi xử lý chuỗi
        thucthilenh(); // Execute the command
    }
*/
// các hàm trên dùng dể đọc chuỗi dữ liệu từ LoRa hoặc RS485
