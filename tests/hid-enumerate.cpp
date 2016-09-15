#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <hidapi.h>
 
#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87
 
int main()
{
    int res;
    #define MAX_STR 255
    wchar_t wstr[MAX_STR];
    hid_device *handle;
 
    struct hid_device_info *devs, *cur_dev;

    hid_init();
    
    devs = hid_enumerate(HTC_ID, VIVE_HMD);
    cur_dev = devs;

    while (cur_dev) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls",
               cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("\n");

        handle = hid_open_path(cur_dev->path);
        if (!handle) {
            printf("unable to open device\n");
            return 1;
        }
        cur_dev = cur_dev->next;
    }
 
    hid_free_enumeration(devs);

    // Read the Manufacturer String
    wstr[0] = 0x0000;
    res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
    if (res < 0)
        printf("Unable to read manufacturer string\n");
    printf("Manufacturer String: %ls\n", wstr);
 
    // Read the Product String
    wstr[0] = 0x0000;
    res = hid_get_product_string(handle, wstr, MAX_STR);
    if (res < 0)
        printf("Unable to read product string\n");
    printf("Product String: %ls\n", wstr);
 
    // Read the Serial Number String
    wstr[0] = 0x0000;
    res = hid_get_serial_number_string(handle, wstr, MAX_STR);
    if (res < 0)
        printf("Unable to read serial number string\n");
    printf("Serial Number String: (%d) %ls", wstr[0], wstr);
    printf("\n");
 
    // Read Indexed String 1
    wstr[0] = 0x0000;
    res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
    if (res < 0)
        printf("Unable to read indexed string 1\n");
    printf("Indexed String 1: %ls\n", wstr);
 
    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(handle, 1);

    //close HID device
    hid_close(handle);
 
    /* Free static HIDAPI objects. */
    hid_exit();
 
    return 0;
}
