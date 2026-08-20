# MEZZA_EXP_IN – Firmware di test scheda (STM32C552CE)

Firmware di collaudo per una scheda basata sul microcontrollore **STM32C552CE** (Cortex-M33), sviluppato con STM32CubeMX/CubeIDE e strutturato come progetto CMake standalone.

Il firmware fa girare **contemporaneamente** tre test verso l'hardware della scheda: lettura degli 8 ingressi analogici 4-20mA, un link seriale UART punto-punto con un'altra scheda, e un link SPI punto-punto con la stessa (o un'altra) scheda remota. Lo scopo è verificare in un solo binario, in fase di bring-up hardware, che tutti i sottosistemi funzionino correttamente insieme, non solo isolatamente.

## Cosa fa

### ADC1 — loop di corrente 4-20mA (PA0…PA7)
- Legge in sequenza gli 8 canali ADC1 (`ADC1_IN0`…`ADC1_IN7`), in polling nel loop principale, ogni 500 ms circa.
- Interpreta ogni ingresso come un loop di corrente 4-20 mA, condizionato a monte in modo che `4 mA → 0.6 V` e `20 mA → 3.0 V` all'ingresso dell'ADC (riferimento `VREF+` esterno a 3.3 V).
- Il risultato (valore grezzo + corrente in µA per canale) è salvato in `adc_results[8]`, ispezionabile da debugger — vedi sezione "Verifica / debug" più sotto.

### USART1 — link punto-punto PING/PONG (PA9=TX, PA10=RX)
- Collegamento seriale interrupt-driven (115200 baud, 8N1) verso un'altra scheda che genera lei stessa un messaggio `PING <n>\r\n` una volta al secondo (nel progetto gemello: USART3 su PB3/PB4).
- Questo firmware riceve il PING, ne estrae il contatore, e risponde `PONG <n>\r\n` sulla stessa linea — tutto a interrupt, nessun polling/timeout.
- USART1 porta **solo** questo traffico protocollare: nessun testo di debug viene mai stampato su questa linea (per non corromperla), a differenza delle prime versioni di questo firmware. Lo stato dello scambio (contatori PING/PONG, eventuali errori di linea, messaggi non riconosciuti) è ispezionabile da debugger.

### SPI2 — echo byte+1 (PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI)
- Questa scheda è **SPI Slave** (mode 0, 8 bit, MSB first, NSS hardware); un'altra scheda è il Master, genera il clock e pilota il chip-select.
- Il master avvia periodicamente un transfer full-duplex da 1 byte con un contatore incrementale; questa scheda risponde con `(byte_ricevuto + 1) mod 256`, mantenendo sempre pronto il prossimo transfer (interrupt-driven, mai in ascolto "spento").
- Nota sul pipelining full-duplex (comportamento atteso, non un bug): il byte che questa scheda trasmette in un dato transfer è la risposta al transfer *precedente*, non a quello in corso — inevitabile in un link sincrono full-duplex.
- Come per l'UART, nessun log viene stampato: lo stato (ultimo byte ricevuto/inviato, contatore transfer, eventuali errori) è ispezionabile da debugger.

Un valore ADC fuori dal range 4–20 mA, o contatori PING/PONG/SPI che non avanzano, sono indicativi di un guasto sul relativo canale/collegamento ed sono utili in fase di collaudo.

## Hardware

| Segnale | Pin | Funzione |
|---|---|---|
| ADC1_IN0…IN7 | PA0…PA7 | Ingressi 4-20mA, canali 1-8 |
| USART1_TX | PA9 | Invio "PONG <n>" verso la scheda remota |
| USART1_RX | PA10 | Ricezione "PING <n>" dalla scheda remota |
| SPI2_NSS | PB12 | Chip-select (pilotato dal Master remoto) |
| SPI2_SCK | PB13 | Clock (generato dal Master remoto) |
| SPI2_MISO | PB14 | Uscita dati verso il Master |
| SPI2_MOSI | PB15 | Ingresso dati dal Master |

Riferimento ADC: `VREF+` esterno, 3.3 V. USART1: 115200 baud, 8N1. SPI2: mode 0 (CPOL=0/CPHA=0), 8 bit, MSB first.

## Verifica / debug

Poiché USART1 è dedicata al protocollo reale con la scheda remota, non c'è output testuale da guardare su un terminale per ADC e SPI: lo stato va ispezionato con un debugger (es. via ST-Link + STM32CubeIDE, oppure letture di memoria dirette con `STM32_Programmer_CLI -c port=SWD mode=HotPlug -r32 <indirizzo> <byte>`, che non resetta la scheda). Variabili principali in `main.c`:

| Variabile | Cosa indica |
|---|---|
| `adc_results[8]` | Ultima lettura raw + corrente (µA) per ciascun canale ADC |
| `comms_ping_rx_count` / `comms_pong_tx_count` | Cicli PING/PONG completati con successo (dovrebbero avanzare in coppia) |
| `comms_error_count` / `comms_last_error_codes` | Errori di linea USART1 (framing/rumore/overrun/...) |
| `comms_unrecognized_count` / `comms_last_unrecognized_msg` | Messaggi ricevuti ma non nel formato "PING n" atteso |
| `spi_xfer_count` | Transfer SPI2 completati con successo |
| `spi_last_rx` / `spi_last_tx` | Ultimo byte ricevuto e prossimo byte di risposta |
| `spi_error_count` / `spi_last_error_codes` | Errori SPI2 (mode fault/overrun/underrun/...) |

## Struttura del progetto

Il progetto usa la nuova architettura STM32Cube (driver HAL/LL unificati per la famiglia STM32C5) generata da STM32CubeMX:

- `main.c` — codice applicativo (punto di ingresso, logica ADC/UART/SPI). È l'unico file pensato per essere modificato liberamente dall'utente.
- `generated/hal/` — codice di inizializzazione periferiche (`mx_*.c/h`), nello stile generato da CubeMX2. `mx_spi2.c/.h` non sono (ancora) gestiti dal tool di codegen del progetto: sono stati aggiunti a mano seguendo lo stesso schema delle altre periferiche.
- `stm32c5xx_drivers/` — driver HAL/LL STM32C5.
- `user_modifiable/` — sorgenti di device/startup/linker script, editabili e uniti (merge) alle rigenerazioni CubeMX.
- `cmake/`, `CMakeLists.txt`, `CMakePresets.json` — configurazione build CMake/Ninja.

## Compilazione

Il progetto richiede **arm-none-eabi-gcc**, **CMake** e **Ninja**. Questi tool non sono necessariamente nel `PATH` di sistema: se è installato STM32CubeIDE, sono disponibili nei suoi plugin (es. `C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\...\tools\bin`).

```sh
cmake --preset debug_GCC_STM32C552CEU6
cmake --build --preset debug_GCC_STM32C552CEU6
```

L'output (`.elf`, `.hex`, `.bin`) viene generato in `build/debug_GCC_STM32C552CEU6/`, pronto per il flashing tramite STM32CubeProgrammer, ST-Link Utility o STM32CubeIDE.

## Note

`build/` non è versionato (vedi `.gitignore`): viene rigenerato ad ogni compilazione.
