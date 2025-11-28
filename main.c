#include <stdio.h>          // Funções de entrada/saída: fopen, fscanf, printf, perror
#include <stdlib.h>         // Funções utilitárias gerais (exit, malloc, etc.)
#include <unistd.h>         // Funções POSIX: usleep(), close(), etc.


// Função principal
int main() {
    FILE *fp;  //ponteiro para arquivo, usado para acessar o ADC.
    unsigned long timestamp = 0;
    int value = 0, scale = 0;
    const char *adc_path = "/sys/kernel/auxadc/adc0";


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
            // float voltage = (value * 3.3f) / scale;
            // printf("ADC0 → tempo=%lu | valor=%d | escala=%d | tensão=%.3f V\n",
            //        timestamp, value, scale, voltage);
            float resistance = (float)value * 10000 / (scale - value); // Calculando a resistencia do potenciometro
            float temperature = (float) 25 + (resistance - 10000) / 1000; // Convertendo para um range de temperatura [25°C, 35°C]
        } else {
            printf("Formato inesperado do ADC\n");
        }


        fclose(fp);
        usleep(200000); // lê a cada 200 ms
    }


    return 0;
}
