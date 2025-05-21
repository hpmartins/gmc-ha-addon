/* sensor:
 * Based on some code from Christoph Haas
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close(), usleep()
#include <time.h>
#include <inttypes.h>
#include <stdbool.h> // For bool, true, false
#include <getopt.h>

#include "cJSON.h"

// Structure for gyroscope data
typedef struct
{
    uint16_t x;    ///< X position (16 bits)
    uint16_t y;    ///< Y position (16 bits)
    uint16_t z;    ///< Z position (16 bits)
    uint8_t event; ///< Event byte (always 0xAA)
} Gyro_Sensor;

// Structure for version info
typedef struct
{
    char model[16];    ///< Device model (e.g. "GMC-300S")
    char revision[16]; ///< Revision (e.g. "Re 1.16")
} Version_Info;

// Private helper functions
static int gmc_write_cmd(int device, const char *cmd)
{
    ssize_t bytes_written = write(device, cmd, strlen(cmd));
    if (bytes_written == -1)
    {
        perror("gmc_write_cmd: write error");
        return -1;
    }
    return (int)bytes_written;
}

static int gmc_read_data(int device, char *buf, int length)
{
    if (!buf || length <= 0)
    {
        return -1;
    }

    int total_read = 0;
    int retries = 3; // Number of retry attempts
    time_t start_time = time(NULL);

    while (total_read < length && retries > 0)
    {
        ssize_t bytes_read = read(device, buf + total_read, length - total_read);

        if (bytes_read > 0)
        {
            total_read += bytes_read;
            if (total_read >= length)
            {
                break; // We got all the data we need
            }
        }
        else if (bytes_read == 0)
        {
            // Timeout occurred - retry a few times
            retries--;
            usleep(50000); // 50ms delay between retries
        }
        else
        {
            // Error occurred
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                retries--;
                usleep(50000); // 50ms delay before retry
                continue;
            }
            perror("gmc_read_data: read error");
            return -1;
        }

        // Check for overall timeout (3 seconds)
        if (time(NULL) - start_time >= 3)
        {
            fprintf(stderr, "gmc_read_data: timeout after 3 seconds\n");
            break;
        }
    }

    return total_read;
}

static bool gmc_flush_input(int device)
{
    // Flush both input and output buffers
    if (tcflush(device, TCIOFLUSH) != 0)
    {
        perror("gmc_flush_input: tcflush error");
        return false;
    }

    // Additional read to clear any remaining data
    char buf[32];
    while (read(device, buf, sizeof(buf)) > 0)
    {
        usleep(1000); // 1ms delay between reads
    }

    return true;
}

static int gmc_send_command_and_read(int device, const char *cmd, char *response, int response_len, const char *func_name)
{
    if (!response || response_len <= 0)
    {
        fprintf(stderr, "%s: invalid response buffer\n", func_name);
        return -1;
    }

    // Flush any pending data before sending command
    if (!gmc_flush_input(device))
    {
        fprintf(stderr, "%s: failed to flush input buffer\n", func_name);
        return -1;
    }

    ssize_t bytes_written = gmc_write_cmd(device, cmd);
    if (bytes_written == -1)
    {
        fprintf(stderr, "%s: write error\n", func_name);
        return -1;
    }
    if (bytes_written != (ssize_t)strlen(cmd))
    {
        fprintf(stderr, "%s: incomplete write, sent %zd of %zu bytes\n",
                func_name, bytes_written, strlen(cmd));
        return -1;
    }

    // Give device time to process command
    usleep(50000); // 50ms delay

    int bytes_read = gmc_read_data(device, response, response_len);
    if (bytes_read == -1)
    {
        fprintf(stderr, "%s: read error\n", func_name);
        return -1;
    }
    if (bytes_read != response_len)
    {
        fprintf(stderr, "%s: incomplete read, got %d of %d bytes\n",
                func_name, bytes_read, response_len);
        return -1;
    }

    return bytes_read;
}

// --- Public API Implementation ---

// Return:   None
bool gmc_set_heartbeat_off(int device)
{
    const char cmd[] = "<HEARTBEAT0>>";
    if (gmc_write_cmd(device, cmd) == (ssize_t)strlen(cmd))
    {
        // Give the device a moment to process the command before flushing
        usleep(10000); // 10ms
        return gmc_flush_input(device);
    }
    return false;
}

static int gmc_open(const char *device, int baud)
{
    int gc_fd = -1;
    struct termios tio;

    // Open the serial port
    gc_fd = open(device, O_RDWR | O_NOCTTY);
    if (gc_fd == -1)
    {
        perror("gmc_open: failed to open serial device");
        return -1;
    }

    // Configure termios settings
    memset(&tio, 0, sizeof(struct termios));

    // Control flags
    tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, Enable receiver, Ignore modem control lines

    // Input flags - disable software flow control and special handling of bytes
    tio.c_iflag = IGNPAR;

    // Output flags - raw output
    tio.c_oflag = 0;

    // Local flags - disable canonical mode, echo, etc.
    tio.c_lflag = 0;

    // Control characters
    tio.c_cc[VMIN] = 0;   // Pure timed read
    tio.c_cc[VTIME] = 10; // 1 second timeout (10 * 0.1s)

    // Determine baud rate constant
    speed_t tio_baud;
    switch (baud)
    {
    case 1200:
        tio_baud = B1200;
        break;
    case 2400:
        tio_baud = B2400;
        break;
    case 4800:
        tio_baud = B4800;
        break;
    case 9600:
        tio_baud = B9600;
        break;
    case 19200:
        tio_baud = B19200;
        break;
    case 38400:
        tio_baud = B38400;
        break;
    case 57600:
        tio_baud = B57600;
        break;
    case 115200:
        tio_baud = B115200;
        break;
    default:
        fprintf(stderr, "gmc_open: warning: unsupported baud rate %d, defaulting to 57600\n", baud);
        tio_baud = B57600;
        break;
    }

    // Set input and output baud rates
    if (cfsetispeed(&tio, tio_baud) != 0 || cfsetospeed(&tio, tio_baud) != 0)
    {
        perror("gmc_open: failed to set baud rate");
        close(gc_fd);
        return -1;
    }

    // Set blocking mode explicitly
    if (fcntl(gc_fd, F_SETFL, 0) == -1)
    {
        perror("gmc_open: failed to set blocking mode");
        close(gc_fd);
        return -1;
    }

    // Apply settings
    if (tcsetattr(gc_fd, TCSANOW, &tio) != 0)
    {
        perror("gmc_open: failed to apply serial port settings");
        close(gc_fd);
        return -1;
    }

    // Initial delay for device stabilization
    usleep(100000); // 100ms

    // Flush buffers
    if (!gmc_flush_input(gc_fd))
    {
        fprintf(stderr, "gmc_open: failed to flush buffers\n");
        close(gc_fd);
        return -1;
    }

    // Additional delay after flush
    usleep(50000); // 50ms

    // Disable heartbeat
    if (!gmc_set_heartbeat_off(gc_fd))
    {
        fprintf(stderr, "gmc_open: failed to disable heartbeat\n");
        close(gc_fd);
        return -1;
    }

    return gc_fd;
}

static void gmc_close(int device)
{
    close(device);
}

static int gmc_get_cpm(int device)
{
    const char cmd[] = "<GETCPM>>";
    unsigned char buf[2] = {0};

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 2, "gmc_get_cpm") != 2)
    {
        return -1;
    }

    return (buf[0] << 8) | buf[1];
}

static float gmc_get_temperature(int device)
{
    const char cmd[] = "<GETTEMP>>";
    unsigned char buf[4] = {0};
    const float ERROR_VALUE = -999.0;

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 4, "gmc_get_temperature") != 4)
    {
        return ERROR_VALUE;
    }

    int sign = (buf[2] == 0) ? 1 : -1;
    float temp = buf[0];
    temp += (float)buf[1] / 100.0;
    return temp * sign;
}

static int gmc_get_volt(int device)
{
    const char cmd[] = "<GETVOLT>>";
    unsigned char buf[1] = {0};
    const int ERROR_VALUE = -1;

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 1, "gmc_get_volt") != 1)
    {
        return ERROR_VALUE;
    }

    return buf[0];
}

static bool gmc_get_serial(int device, char *serialNum)
{
    const char cmd[] = "<GETSERIAL>>";
    unsigned char buf[7] = {0};

    if (!serialNum)
    {
        fprintf(stderr, "gmc_get_serial: null serial number pointer\n");
        return false;
    }

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 7, "gmc_get_serial") != 7)
    {
        return false;
    }

    sprintf(serialNum, "%02X%02X%02X%02X%02X%02X%02X",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    return true;
}

static bool gmc_get_version_info(int device, Version_Info *version_info)
{
    const char cmd[] = "<GETVER>>";
    char buf[16] = {0};            // Buffer for 15 chars + null terminator
    const size_t MODEL_LENGTH = 8; // First 8 characters are the model

    if (!version_info)
    {
        fprintf(stderr, "gmc_get_version_info: null version info pointer\n");
        return false;
    }

    if (gmc_send_command_and_read(device, cmd, buf, 15, "gmc_get_version_info") != 15)
    {
        return false;
    }

    buf[15] = '\0'; // Ensure null termination

    // Copy model (first 8 characters)
    memcpy(version_info->model, buf, MODEL_LENGTH);
    version_info->model[MODEL_LENGTH] = '\0';

    // Copy revision (everything after model)
    size_t rev_len = strlen(buf + MODEL_LENGTH);
    if (rev_len >= sizeof(version_info->revision))
    {
        rev_len = sizeof(version_info->revision) - 1;
    }
    memcpy(version_info->revision, buf + MODEL_LENGTH, rev_len);
    version_info->revision[rev_len] = '\0';

    return true;
}

static bool gmc_get_gyro(int device, Gyro_Sensor *gyro)
{
    const char cmd[] = "<GETGYRO>>";
    unsigned char buf[7] = {0};

    if (!gyro)
    {
        fprintf(stderr, "gmc_get_gyro: null gyro pointer\n");
        return false;
    }

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 7, "gmc_get_gyro") != 7)
    {
        return false;
    }

    if (buf[6] != 0xAA)
    {
        fprintf(stderr, "gmc_get_gyro: invalid event byte 0x%02X\n", buf[6]);
        return false;
    }

    gyro->x = (buf[0] << 8) | buf[1];
    gyro->y = (buf[2] << 8) | buf[3];
    gyro->z = (buf[4] << 8) | buf[5];
    gyro->event = buf[6];
    return true;
}

// <SETDATETIME[YYMMDDHHMMSS]>>
bool gmc_set_datetime(int device)
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    unsigned char cmd[20] = "<SETDATETIME"; // Command header
    unsigned char response[1] = {0};

    // Flush any pending data before sending command
    if (!gmc_flush_input(device))
    {
        fprintf(stderr, "gmc_set_datetime: failed to flush input buffer\n");
        return false;
    }

    // Build command with current date/time
    cmd[12] = tm.tm_year % 100;
    cmd[13] = tm.tm_mon + 1;
    cmd[14] = tm.tm_mday;
    cmd[15] = tm.tm_hour;
    cmd[16] = tm.tm_min;
    cmd[17] = tm.tm_sec;
    cmd[18] = '>';
    cmd[19] = '>';

    ssize_t bytes_written = write(device, cmd, 20);
    if (bytes_written == -1)
    {
        fprintf(stderr, "gmc_set_datetime: write error: %s\n", strerror(errno));
        return false;
    }
    if (bytes_written != 20)
    {
        fprintf(stderr, "gmc_set_datetime: incomplete write, sent %zd of 20 bytes\n", bytes_written);
        return false;
    }

    // Give device time to process command
    usleep(50000); // 50ms delay

    int bytes_read = gmc_read_data(device, (char *)response, 1);
    if (bytes_read == -1)
    {
        fprintf(stderr, "gmc_set_datetime: read error\n");
        return false;
    }
    if (bytes_read != 1)
    {
        fprintf(stderr, "gmc_set_datetime: incomplete read, got %d of 1 bytes\n", bytes_read);
        return false;
    }

    if (response[0] != 0xAA)
    {
        fprintf(stderr, "gmc_set_datetime: invalid response byte 0x%02X\n", response[0]);
        return false;
    }

    return true;
}

static float gmc_get_unit_conversion_factor(int device)
{
    const char cmd[] = "<GETCFG>>";
    unsigned char buf[26] = {0};
    const float ERROR_VALUE = -1.0;

    if (gmc_send_command_and_read(device, cmd, (char *)buf, 26, "gmc_get_unit_conversion_factor") != 26)
    {
        fprintf(stderr, "gmc_get_unit_conversion_factor: failed to read configuration data\n");
        return ERROR_VALUE;
    }

    // Extract CPM values (16-bit big-endian integers)
    uint16_t cpm1 = (buf[8] << 8) | buf[9];
    uint16_t cpm2 = (buf[14] << 8) | buf[15];
    uint16_t cpm3 = (buf[20] << 8) | buf[21];

    // Extract μSv/h values (32-bit little-endian floats)
    float usv1, usv2, usv3;
    memcpy(&usv1, &buf[10], 4);
    memcpy(&usv2, &buf[16], 4);
    memcpy(&usv3, &buf[22], 4);

    // Calculate average conversion factor
    if (cpm1 == 0 || cpm2 == 0 || cpm3 == 0)
    {
        fprintf(stderr, "gmc_get_unit_conversion_factor: invalid CPM values in calibration data\n");
        return ERROR_VALUE;
    }

    float factor1 = usv1 / cpm1;
    float factor2 = usv2 / cpm2;
    float factor3 = usv3 / cpm3;

    return (factor1 + factor2 + factor3) / 3.0;
}

static void print_usage(const char *program_name)
{
    fprintf(stderr, "Usage: %s <serial_device> [options]\n", program_name);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -h, --help              Show this help message\n");
    fprintf(stderr, "  -a, --all               Show all data (default)\n");
    fprintf(stderr, "  -c, --cpm               Show CPM (Counts Per Minute)\n");
    fprintf(stderr, "  -s, --serial            Show serial number\n");
    fprintf(stderr, "  -t, --temperature       Show temperature\n");
    fprintf(stderr, "  -v, --voltage           Show voltage\n");
    fprintf(stderr, "  -g, --gyro              Show gyroscope data\n");
    fprintf(stderr, "  -m, --model             Show device model\n");
    fprintf(stderr, "  -r, --revision          Show firmware revision\n");
    fprintf(stderr, "  -f, --conversion-factor Show conversion factor\n");
    fprintf(stderr, "  -b, --baudrate <value>  Set baudrate (default: 57600)\n");
    fprintf(stderr, "\nExample: %s /dev/ttyUSB0 --cpm --temperature --baudrate 115200\n", program_name);
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage(argv[0]);
        return 1;
    }

    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"all", no_argument, 0, 'a'},
        {"cpm", no_argument, 0, 'c'},
        {"serial", no_argument, 0, 's'},
        {"temperature", no_argument, 0, 't'},
        {"voltage", no_argument, 0, 'v'},
        {"gyro", no_argument, 0, 'g'},
        {"model", no_argument, 0, 'm'},
        {"revision", no_argument, 0, 'r'},
        {"conversion-factor", no_argument, 0, 'f'},
        {"baudrate", required_argument, 0, 'b'},
        {0, 0, 0, 0}};

    int show_all = 1; // Default to showing all if no flags specified
    int show_cpm = 0;
    int show_serial = 0;
    int show_temp = 0;
    int show_volt = 0;
    int show_gyro = 0;
    int show_model = 0;
    int show_revision = 0;
    int show_conversion_factor = 0;

    int option_index = 0;
    int opt;
    char *serial_device = NULL;
    int baudrate = 57600; // Default baud rate

    // Parse command line options
    while ((opt = getopt_long(argc, argv, "hacstvgmrfb:", long_options, &option_index)) != -1)
    {
        switch (opt)
        {
        case 'h':
            print_usage(argv[0]);
            return 0;
        case 'a':
            show_all = 1;
            break;
        case 'c':
            show_all = 0;
            show_cpm = 1;
            break;
        case 's':
            show_all = 0;
            show_serial = 1;
            break;
        case 't':
            show_all = 0;
            show_temp = 1;
            break;
        case 'v':
            show_all = 0;
            show_volt = 1;
            break;
        case 'g':
            show_all = 0;
            show_gyro = 1;
            break;
        case 'm':
            show_all = 0;
            show_model = 1;
            break;
        case 'r':
            show_all = 0;
            show_revision = 1;
            break;
        case 'f':
            show_all = 0;
            show_conversion_factor = 1;
            break;
        case 'b':
        {
            char *p;
            long parsed_baud = strtol(optarg, &p, 10);
            if (*p != '\0' || errno == ERANGE)
            {
                fprintf(stderr, "Error: Invalid baudrate '%s'\n", optarg);
                return 1;
            }
            baudrate = (int)parsed_baud;
            break;
        }
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    // Get serial device from remaining arguments
    if (optind < argc)
    {
        serial_device = argv[optind];
    }
    else
    {
        fprintf(stderr, "Error: Serial device not specified.\n");
        print_usage(argv[0]);
        return 1;
    }

    int serial_port = gmc_open(serial_device, baudrate);
    if (serial_port == -1)
    {
        fprintf(stderr, "Error: Cannot open specified serial device %s\n", serial_device);
        return 1;
    }

    // Create JSON output
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        fprintf(stderr, "Error: cJSON_CreateObject failed\n");
        gmc_close(serial_port);
        return 1;
    }

    cJSON *attributes = cJSON_CreateObject();
    if (!attributes)
    {
        fprintf(stderr, "Error: cJSON_CreateObject (attributes) failed\n");
        cJSON_Delete(root);
        gmc_close(serial_port);
        return 1;
    }
    cJSON_AddItemToObject(root, "attributes", attributes);

    // Get and add requested data
    if (show_all || show_cpm)
    {
        int cpm = gmc_get_cpm(serial_port);
        cJSON_AddNumberToObject(root, "state", cpm);
        cJSON_AddStringToObject(attributes, "unit_of_measurement", "cpm");
    }

    if (show_all || show_model || show_revision)
    {
        Version_Info version_info;
        if (gmc_get_version_info(serial_port, &version_info))
        {
            if (show_all || show_model)
            {
                cJSON_AddStringToObject(attributes, "model", version_info.model);
            }
            if (show_all || show_revision)
            {
                cJSON_AddStringToObject(attributes, "revision", version_info.revision);
            }
        }
        else
        {
            if (show_all || show_model)
            {
                cJSON_AddStringToObject(attributes, "model", "N/A");
            }
            if (show_all || show_revision)
            {
                cJSON_AddStringToObject(attributes, "revision", "N/A");
            }
        }
    }

    if (show_all || show_serial)
    {
        char serialNumber[20];
        if (gmc_get_serial(serial_port, serialNumber))
        {
            cJSON_AddStringToObject(attributes, "serial", serialNumber);
        }
        else
        {
            cJSON_AddStringToObject(attributes, "serial", "N/A");
        }
    }

    if (show_all || show_temp)
    {
        float temp = gmc_get_temperature(serial_port);
        if (temp != -999.0)
        {
            cJSON_AddNumberToObject(attributes, "temp", temp);
        }
        else
        {
            cJSON_AddStringToObject(attributes, "temp", "N/A");
        }
    }

    if (show_all || show_volt)
    {
        float volt = (float)gmc_get_volt(serial_port) / 10.0;
        if (volt != -0.1)
        {
            cJSON_AddNumberToObject(attributes, "volt", volt);
        }
        else
        {
            cJSON_AddStringToObject(attributes, "volt", "N/A");
        }
    }

    if (show_all || show_gyro)
    {
        Gyro_Sensor gyro;
        if (gmc_get_gyro(serial_port, &gyro))
        {
            cJSON_AddNumberToObject(attributes, "x", gyro.x);
            cJSON_AddNumberToObject(attributes, "y", gyro.y);
            cJSON_AddNumberToObject(attributes, "z", gyro.z);
        }
        else
        {
            cJSON_AddStringToObject(attributes, "x", "N/A");
            cJSON_AddStringToObject(attributes, "y", "N/A");
            cJSON_AddStringToObject(attributes, "z", "N/A");
        }
    }

    if (show_all || show_conversion_factor)
    {
        float conversion_factor = gmc_get_unit_conversion_factor(serial_port);
        cJSON_AddNumberToObject(attributes, "conversion_factor", conversion_factor);
    }

    // Print JSON output
    char *json_output = cJSON_Print(root);
    if (json_output)
    {
        printf("%s\n", json_output);
        free(json_output);
    }
    else
    {
        fprintf(stderr, "Error: Failed to print cJSON object.\n");
    }

    cJSON_Delete(root);
    gmc_close(serial_port);

    return 0;
}