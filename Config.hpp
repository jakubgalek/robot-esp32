#pragma once

// ---------------------------------------------------------------------------
// Wspólna konfiguracja progów bezpieczeństwa (wcześniej każde miejsce w kodzie
// miało własne wartości: WWW 10 cm, checkDriveConditions 15 cm, BLE 35 cm).
// Wszystkie odległości w cm.
// ---------------------------------------------------------------------------

// Ręczna jazda do przodu: stop, gdy przeszkoda bliżej niż...
#define STOP_FRONT_SIDE_CM    4    // czujniki przód-lewo i przód-prawo
#define STOP_FRONT_CENTER_CM  15   // czujnik przód-środek

// Czujniki przeszkód: dane starsze niż tyle ms uznajemy za nieaktualne.
// Robot nie rusza (a jadący staje), dopóki dane nie są świeże.
#define RTC_ENABLED           1        // 0 = nie używaj zegara DS3231 (nic z niego nie jest czytane z magistrali I2C; na ekranie godzina 00:00)
#define I2C_CLOCK_HZ          400000   // wszystkie układy na magistrali (VL53L0X, MPU6050, PCF8574, INA3221, BMP180, DS3231) obsługują 400 kHz
#define SENSOR_MAX_AGE_MS     500
#define SENSOR_MAX_AGE_REAR_MS 600   // tył: cofamy wolno, więc tolerujemy starsze dane (mniej fałszywych stopów)

// ---------------------------------------------------------------------------
// GEOMETRIA ROBOTA I MAPY  (wszystko w cm / stopniach)
// Pozycja robota (0,0) = punkt między kołami, środek obrotu. Oś X w prawo, Y do przodu
// (przy starcie robot patrzy "w górę mapy"). Kąt: 0 = oś X, rośnie przeciwnie do ruchu
// wskazówek zegara (skręt w lewo = większy kąt). Start: 90 = do przodu.
// ---------------------------------------------------------------------------
#define ROBOT_LENGTH_CM        26.0f   // wymiar wzdłuż jazdy
#define ROBOT_WIDTH_CM         15.3f   // szerokość razem z kołami (obrys do kolizji i mapy)
#define ROBOT_BODY_WIDTH_CM    14.2f   // szerokość samego korpusu (do rysunków)
#define WHEEL_DIAMETER_CM      7.1f    // średnica koła (droga na impuls enkodera wynika z niej: obwód 22,3 cm / 40 impulsów)
#define WHEEL_AXLE_FROM_FRONT_CM 13.0f // ZGADYWANE (oś kół w połowie długości) - zmierz: od przodu robota do osi kół
#define ROBOT_START_HEADING    90.0f

// Położenie czujników serwa względem środka robota (X do przodu, Y w lewo), cm.
// Zmierz linijką i popraw - to wprost przesuwa początek promieni na mapie.
// Bęben serwa: czujnik 0 (do przodu) jest 1,5 cm od przodu robota, czujnik 1 (do tyłu) 3,8 cm dalej, oba na środku szerokości.
// Oś obrotu bębna leży więc w połowie między nimi (3,4 cm od przodu), a czujniki są 1,9 cm od osi.
#define SERVO_AXIS_FROM_FRONT_CM     3.4f
#define SERVO_SENSOR_RADIUS_CM       1.9f   // dodawane do zmierzonej odległości (pomiar liczony od osi bębna)
#define SERVO_SENSOR_OFFSET_FWD_CM   (WHEEL_AXLE_FROM_FRONT_CM - SERVO_AXIS_FROM_FRONT_CM)
#define SERVO_SENSOR_OFFSET_LEFT_CM  0.0f

// Serwo: kąt 0 = czujnik 0 patrzy prosto do przodu, czujnik 1 prosto do tyłu.
// +1: rosnący kąt serwa obraca czujniki w LEWO (przeciwnie do zegara), -1: w prawo.
// Sprawdzisz: po skanie ściana po lewej stronie robota ma być po lewej na mapie.
#define SERVO_DIRECTION        1
#define SERVO_ZERO_OFFSET_DEG  0.0f    // poprawka, jeśli "0" serwa nie jest idealnie prosto

// Odometria
// Droga na jeden impuls enkodera [cm]. Wartość startowa pochodzi z obwodu koła;
// zmierz rzeczywistą drogę (przycisk "Kalibracja drogi" w panelu) - wynik zapisuje się w pamięci.
#define ENCODER_CM_PER_PULSE_DEFAULT  0.5575f
#define GYRO_SCALE             1.0f    // mnożnik żyroskopu (1.0 = bez korekty)
#define GYRO_DEADBAND_DPS      0.4f    // poniżej tej prędkości kątowej uznajemy, że się nie obracamy
#define GYRO_DEADBAND_STRAIGHT_DPS 0.3f   // jazda prosto: reagujemy na wolniejsze zbaczanie niż przy obrocie
#define ODO_SETTLE_MS          350     // tyle ms po wyłączeniu silników nadal liczymy dobieg

// Mapa: siatka zajętości
#define MAP_CELLS              200     // preferowana liczba komórek na bok (200 x 5 cm = 10 x 10 m, start w środku).
                                       // Gdy brakuje pamięci, robot sam zmniejsza do 160, potem 128 (komunikat w konsoli).
#define MAP_CELL_CM            5       // rozmiar komórki; 5 cm = robot to 5 x 3 komórki
#define MAP_MAX_RANGE_CM       120     // dalej czujnik VL53L0X jest niewiarygodny
#define MAP_MIN_RANGE_CM       3
#define MAP_NOECHO_FREE_CM     50      // brak echa: oznacz tylko tyle jako wolne (słabo)

// Skan 360 stopni na postoju: serwo 0..180 co SCAN_STEP_DEG, dwa czujniki = pełny krąg
#define SCAN_STEP_DEG          3
#define SCAN_SETTLE_MS         45      // czas na dojście serwa do kąta przed pomiarem

// Dopasowanie skanu do mapy (korekta dryfu odometrii po każdym skanie)
#define WALL_FIX_ENABLED       1       // korekta kierunku względem ścian pokoju (większość ścian jest do siebie prostopadła)
#define WALL_FIX_MIN_CONF      0.45f   // minimalna "pewność" orientacji ścian w skanie (0..1)
#define WALL_FIX_MAX_DEG       25.0f   // większych różnic nie korygujemy (to już nie dryf, tylko co innego)
#define SCAN_MATCH_ENABLED     1
#define SCAN_MATCH_MIN_HITS    30      // minimalna liczba punktów w skanie
#define SCAN_MATCH_MAX_SHIFT_CM 10     // maks. korekta położenia w jednym skanie
#define SCAN_MATCH_MAX_TURN_DEG 6     // maks. korekta kąta w jednym skanie

// Czujniki stałe (2..7) jako dodatkowe źródło przeszkód na mapie podczas jazdy.
// Geometria poniżej jest ZGADYWANA z Twojego opisu - zmierz i popraw w tablicy FIXED_SENSORS
// (Scan.hpp): przesunięcie do przodu / w lewo od środka robota [cm] i kierunek patrzenia
// względem osi robota [stopnie, + = w lewo]. Zła geometria = duchy na mapie; wtedy wyłącz (0).
#define FIXED_SENSOR_MAPPING   1
#define FIXED_MAP_MAX_CM       60     // dalsze pomiary ignorujemy (stałe czujniki mają szeroki stożek)
#define FIXED_MAP_MIN_CM       4

// Jazda automatyczna (eksploracja): skan co tyle przejechanych cm (poza skanem w każdym celu)
#define AUTO_SCAN_EVERY_CM    60      // domyślnie; w Ustawieniach na ekranie można zmienić (gScanEveryCm)

// Zapis mapy w SPIFFS (nie częściej niż co tyle ms)
#define MAP_SAVE_MIN_INTERVAL_MS 8000

// ---------------------------------------------------------------------------
// Eksploracja (jazda automatyczna z mapą)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Jazda wzdłuż lewej ściany (bez skanowania mapy) - WallFollow.hpp
// Czujniki boczne lewe: nr 2 (lewy przód) i nr 7 (lewy tył). Odległość między nimi jest ZGADYWANA - zmierz i popraw.
// ---------------------------------------------------------------------------
#define WALLF_SENSOR_SPACING_CM 16.0f   // odległość wzdłuż robota między czujnikiem 2 a 7
#define WALLF_TARGET_CM         10.0f   // żądana odległość lewego boku robota od ściany (odczyt czujnika bocznego)
#define WALLF_SEEN_CM           45      // dalej niż to = ściany nie widać (otwarcie, róg zewnętrzny)
#define WALLF_SEG_CM            15.0f   // odcinek jazdy między kolejnymi korektami
#define WALLF_FRONT_TURN_CM     26      // przeszkoda z przodu bliżej niż to = róg wewnętrzny, skręt w prawo
#define WALLF_CORNER_CM         10.0f   // za rogiem zewnętrznym: tyle dojechać, zanim skręcimy w lewo

// ---------------------------------------------------------------------------
// Drugi przebieg skanowania - Coverage.hpp
// ---------------------------------------------------------------------------
#define REFINE_GRID_CM         70.0f   // drugi przebieg: skan co ok. tyle cm po całym wolnym obszarze (punkty inne niż w pierwszym przejeździe)
#define REFINE_CLEAR_CM        25.0f
#define EXPLORE_SECOND_PASS    0       // 1 = po zmapowaniu pokoju robot automatycznie robi drugi przebieg (przycisk "Drugi przebieg" działa niezależnie od tego)

static float gScanEveryCm = AUTO_SCAN_EVERY_CM;   // aktualny odstęp skanów podczas jazdy automatycznej [cm] (ustawiany z ekranu, zapisywany w NVS)

// ---------------------------------------------------------------------------
// Opcjonalna karta SD (SDCard.hpp). Robot działa tak samo bez niej; karta dodaje:
//   - dziennik zdarzeń w pliku /robot/log.txt (do czytania na komputerze),
//   - automatyczną kopię mapy (/robot/map_last.bin) - wraca po "Sketch Data Upload", który kasuje flash,
//   - bibliotekę zapisanych map (/robot/maps/*.bin + podgląd *.pgm).
// Piny 13, 14, 15, 32 muszą być wolne (nie używane przez ekran, dotyk ani silniki).
// ---------------------------------------------------------------------------
// Strefa czasowa dla synchronizacji z internetu (Polska: czas zimowy/letni automatycznie)
#define NTP_TZ "CET-1CEST,M3.5.0,M10.5.0/3"

#define SD_ENABLED   1       // 0 = całkowicie wyłącza obsługę SD (oszczędza ok. 40-60 KB flash)
// Karta SD na osobnej magistrali HSPI (nie dzieli SPI z ekranem): MOSI 13, MISO 32, CLK 14, CS 15
#define SD_MOSI_PIN  13
#define SD_MISO_PIN  32
#define SD_SCK_PIN   14
#define SD_CS_PIN    15
#define SD_FREQ_HZ   4000000
