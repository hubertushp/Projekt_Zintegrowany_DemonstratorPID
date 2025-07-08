# KOD DO ESP32 DO OBSŁUGI DEMONSTRATORA PID

Kod main.cpp znajduje się w folderze src/main.cpp

## Wymagania wstępne

Przed rozpoczęciem upewnij się, że masz:

- [Visual Studio Code](https://code.visualstudio.com/)
- Rozszerzenie [PlatformIO IDE](https://platformio.org/platformio-ide)

## Instalacja projektu

### Pobranie repozytorium

Klonuj repozytorium:

```bash
git clone <adres-twojego-repozytorium>
cd <nazwa-folderu-repozytorium>
```
Lub pobierz w formie .zip z Code -> Local .zip

## Konfiguracja projektu

### Otwórz projekt w VS Code

- Uruchom Visual Studio Code.
- Kliknij na ikone PlatformIO
- Wybierz z lewej strony z listy Quick Acces Open
- `Open Projekt` → wskaż katalog projektu.


## Kompilacja i wgranie programu

### Kompilacja kodu

Przed pierwszym skompilowaniem:

- Kliknij ikonę PlatformIO.
- Wybierz opcję **Build Filesystem Image** a następnie **Upload Filesystem Image**
- Wybierz opcję **Build** lub uruchom komendę:

```bash
pio run
```

### Wgrywanie programu

- Podłącz ESP32 do komputera za pomocą USB.
- Kliknij opcję **Upload** lub użyj komendy:

```bash
pio run --target upload
```

## Monitorowanie ESP32

Aby wyświetlić logi z ESP32:

- Uruchom **Serial Monitor** lub wpisz komendę:

```bash
pio device monitor
```


## Rozwiązywanie problemów

- **Problem z wykrywaniem ESP32:**
  - Sprawdź połączenie USB.
  - Zainstaluj sterowniki USB [CP2102](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers).


