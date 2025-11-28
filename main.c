#include <stdio.h>              // Funções de entrada/saída
#include <stdlib.h>             // Funções utilitárias (exit, malloc)
#include <fcntl.h>              // Controle de arquivos (open, O_RDWR, O_NOCTTY, etc.)
#include <unistd.h>             // Funções POSIX (read, write, close)
#include <sys/ioctl.h>          // Habilita o uso da syscall ioctl() (Input/Output Control), que envia comandos específicos a drivers de dispositivos.
#include <linux/i2c-dev.h>      // Fornece definições, constantes e macros específicas para o driver de dispositivos I²C do Linux.
#include <linux/i2c.h>          // Complementa <linux/i2c-dev.h>, trazendo estruturas e flags internas do protocolo I²C.          
#include <stdint.h>             // Define tipos de inteiros com tamanhos explícitos e portáveis, evitando ambiguidades entre arquiteturas (32 bits, 64 bits etc.).


// Definições iniciais - variáveis globais e macros
typedef unsigned char u8;
int i2c_fd = -1;
const char *i2c_fname = "/dev/i2c-2"; // barramento I2C da Labrador


// BH1750
#define _BH1750_I2C_ADDR 0x23       // Device's I2C address
const uint8_t _POWER_ON_C = 0x01;   // Power on command
const uint8_t _CONT_HRES_C = 0x10;  // Modo de alta resolução (1 lux)
const uint8_t _CONT_HRES2_C = 0x11; // Modo de alta resolução 2 (0.5 lux)
const uint8_t _CONT_LRES_C = 0x13;  // Modo de baixa resolução (4 lux)

// BMP280
#define BMP280_I2C_ADDR _u(0x76)

#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

#define NUM_CALIB_PARAMS 24

struct bmp280_calib_param {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};

// Prototipos de funções
int i2c_init(void);
int i2c_write(u8 slave_addr, u8 reg, u8 data);
int i2c_read_bytes(u8 slave_addr, u8 reg, u8 *buffer, u8 length);
void bh1750_init(void);
void bh1750_read_lux(float *lux);
void bmp280_init(void);
void bmp280_read_temp_pressure(float *temperature, float *pressure);
int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param *params);
int32_t bmp280_convert_temp(int32_t bmp_tmp, struct bmp280_calib_param *params);
int32_t bmp280_convert_pressure(int32_t bmp_press, int32_t temp, struct bmp280_calib_param* params);
void bmp280_get_calib_params(struct bmp280_calib_param *calib_params);



// Abre o barramento I²C em modo leitura/escrita (O_RDWR).
int i2c_init(void) {
    if ((i2c_fd = open(i2c_fname, O_RDWR)) < 0) {
        perror("Erro ao abrir I2C");
        return -1;
    }
    return i2c_fd;
}


// Envia dois bytes (endereço do registrador e valor) para um dispositivo escravo no barramento I²C.
int i2c_write(u8 slave_addr, u8 reg, u8 data) {
    u8 outbuf[2] = {reg, data};
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;


    // Configuração da mensagem I2C
    msg.addr  = slave_addr;
    msg.flags = 0;
    msg.len   = 2;
    msg.buf   = outbuf;


    // Agrupamento da transação
    msgset.msgs  = &msg;
    msgset.nmsgs = 1;


    // Execução da operação via ioctl()
    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("Erro no write");
        return -1;
    }
    return 0;
}


// Leitura genérica de N bytes consecutivos a partir de um registrador
/*
[Start] -> [Endereço Escravo + Write] -> [Registrador] -> [Restart]
        -> [Endereço Escravo + Read]  -> [Lê N bytes] -> [Stop]
*/
int i2c_read_bytes(u8 slave_addr, u8 reg, u8 *buffer, u8 length) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;


    msgs[0].addr  = slave_addr;
    msgs[0].flags = 0;              // Escrita
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;


    msgs[1].addr  = slave_addr;
    msgs[1].flags = I2C_M_RD;       // Leitura
    msgs[1].len   = length;
    msgs[1].buf   = buffer;


    msgset.msgs  = msgs;
    msgset.nmsgs = 2;


    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("Erro na leitura I2C");
        return -1;
    }


    return 0;
}


// Funções do BH1750
void bh1750_init(void){
    i2c_write(_BH1750_I2C_ADDR, _POWER_ON_C, 0);
    i2c_write(_BH1750_I2C_ADDR, _CONT_HRES_C, 0);
}

void bh1750_read_lux(float *lux){
    i2c_write(_BH1750_I2C_ADDR, _CONT_HRES_C, 0);
    usleep(200000); // Espera para  completar a medição

    u8 buffer[2];
    i2c_read_bytes(_BH1750_I2C_ADDR, 0, buffer, 2);
    *lux = ((buffer[0] << 8) | buffer[1]) / 1.2; // Conversão para lux
}


// Funções do BMP280
void bmp280_init(void){
    u8 buf[2];
    const u8 reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;

    i2c_write(BMP280_I2C_ADDR, buf[0], buf[1]);

    const u8 reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | 0x03;
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write(BMP280_I2C_ADDR, buf[0], buf[1]);
}

void bmp280_read_temp_pressure(float *temperature, float *pressure){
    u8 buffer[6];
    u8 reg = REG_PRESSURE_MSB;
    i2c_write(BMP280_I2C_ADDR, reg, 1);
    i2c_read_bytes(BMP280_I2C_ADDR, REG_PRESSURE_MSB, buffer, 6);

    int32_t adc_P = (int32_t)((((uint32_t)(buffer[0]) << 12) | ((uint32_t)(buffer[1]) << 4) | ((uint32_t)(buffer[2]) >> 4)));
    int32_t adc_T = (int32_t)((((uint32_t)(buffer[3]) << 12) | ((uint32_t)(buffer[4]) << 4) | ((uint32_t)(buffer[5]) >> 4)));

    struct bmp280_calib_param calib_params;
    bmp280_get_calib_params(&calib_params);
    *temperature = bmp280_convert_temp(adc_T, &calib_params) / 100.0f;
    *pressure = bmp280_convert_pressure(adc_P, adc_T, &calib_params) / 1000.0f;
}

int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param *params){
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t bmp_tmp, struct bmp280_calib_param *params){
    int32_t t_fine = bmp280_convert(bmp_tmp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t bmp_press, int32_t temp, struct bmp280_calib_param* params){
    int32_t t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

void bmp280_get_calib_params(struct bmp280_calib_param *calib_params){
    u8 buffer[NUM_CALIB_PARAMS];
    u8 reg = REG_DIG_T1_LSB;
    i2c_write(BMP280_I2C_ADDR, reg, 1);
    i2c_read_bytes(BMP280_I2C_ADDR, REG_DIG_T1_LSB, buffer, NUM_CALIB_PARAMS);

    calib_params->dig_t1 = (uint16_t)(buffer[0] | (buffer[1] << 8));
    calib_params->dig_t2 = (int16_t)(buffer[2] | (buffer[3] << 8));
    calib_params->dig_t3 = (int16_t)(buffer[4] | (buffer[5] << 8));
    calib_params->dig_p1 = (uint16_t)(buffer[6] | (buffer[7] << 8));
    calib_params->dig_p2 = (int16_t)(buffer[8] | (buffer[9] << 8));
    calib_params->dig_p3 = (int16_t)(buffer[10] | (buffer[11] << 8));
    calib_params->dig_p4 = (int16_t)(buffer[12] | (buffer[13] << 8));
    calib_params->dig_p5 = (int16_t)(buffer[14] | (buffer[15] << 8));
    calib_params->dig_p6 = (int16_t)(buffer[16] | (buffer[17] << 8));
    calib_params->dig_p7 = (int16_t)(buffer[18] | (buffer[19] << 8));
    calib_params->dig_p8 = (int16_t)(buffer[20] | (buffer[21] << 8));
    calib_params->dig_p9 = (int16_t)(buffer[22] | (buffer[23] << 8));
}


// Função principal
int main() {
    FILE *fp;  //ponteiro para arquivo, usado para acessar o ADC.
    unsigned long timestamp = 0;
    int value = 0, scale = 0;
    const char *adc_path = "/sys/kernel/auxadc/adc0";

    float temperature_adc = 0.0f;
    float temperature_bmp = 0.0f;
    float pressure_bmp = 0.0f;
    float lux = 0.0f;


    i2c_init();
    bmp280_init();
    bmp280_init();

    // Loop de leitura contínua
    while (1) {
        /*Abre o arquivo /sys/kernel/auxadc/adc0 em modo leitura ("r")
        O kernel fornece uma leitura “instantânea” (snapshot) do ADC.
        Caso o driver ou o caminho não exista, fopen() retorna NULL, e o erro é exibido com perror().
        */
        fp = fopen(adc_path, "r");
        if (fp == NULL) {
            perror("Erro ao abrir ADC");
            return 1;
        }
        /*
        Lê formato: <timestamp> <value> / <scale>
        fscanf() lê os três valores numéricos do arquivo aberto.
        */
        if (fscanf(fp, "%lu %d / %d", &timestamp, &value, &scale) == 3) {
            float resistance = (float)value * 10000 / (scale - value); // Calculando a resistencia do potenciometro
            temperature_adc = (float) 25 + (resistance - 10000) / 1000; // Convertendo para um range de temperatura [25°C, 35°C]
        } else {
            printf("Formato inesperado do ADC\n");
        }

        // Leitura do BH1750
        bh1750_read_lux(&lux);
        // Leitura do BMP280
        bmp280_read_temp_pressure(&temperature_bmp, &pressure_bmp);


        // Logs
        printf("Luminosidade: %.2f lux\n", lux);
        printf("Temperatura BMP280: %.2f °C\n", temperature_bmp);
        printf("Pressão BMP280: %.2f kPa\n", pressure_bmp);
        printf("Temperatura ADC: %.2f °C\n", temperature_adc);
        printf("---------------------------\n");

        fclose(fp);
        usleep(200000); // lê a cada 200 ms
    }

    close(i2c_fd);
    return 0;
}
