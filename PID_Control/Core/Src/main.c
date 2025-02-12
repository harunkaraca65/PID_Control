/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68<<1 // MPU6050 sensörünün I2C adresi. 0x68 adresini sola 1 bit kaydırarak (0xD0) elde edilir.
#define PWR_MGMT_1_REG 0x6B // Güç yönetimi kaydı 1'in adresi. Sensörün çalışma modunu ayarlamak için kullanılır.
#define SMPLRT_DIV_REG 0x19 // Örnekleme hızı bölen kaydının adresi. Sensörün örnekleme hızını ayarlamak için kullanılır.
#define GYRO_CNFG_REG 0x1B // Jiroskop yapılandırma kaydının adresi. Jiroskopun hassasiyetini ayarlamak için kullanılır.
#define ACC_CNFG_REG 0x1C // İvmeölçer yapılandırma kaydının adresi. İvmeölçerin hassasiyetini ayarlamak için kullanılır.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
uint8_t data; // Genel amaçlı bir bayt (8 bit) veri saklamak için kullanılır.
uint8_t buffer[2], cuffer[2], tuffer[2]; // I2C veri alışverişi için kullanılan bayt dizileri.
int16_t gyro_raw[3], acc_raw[3]; // Ham jiroskop ve ivmeölçer verilerini saklamak için kullanılan 16 bitlik tam sayı dizileri.
float gyro_cal[3]; // Jiroskop kalibrasyon değerlerini saklamak için kullanılan kayan noktalı sayı dizisi.
int16_t acc_total_vector; // İvmeölçer verilerinin toplam vektör büyüklüğünü saklamak için kullanılan 16 bitlik tam sayı.
float angle_pitch_gyro, angle_roll_gyro; // Jiroskop verilerine dayalı olarak hesaplanan pitch ve roll açıları.
float angle_pitch_acc, angle_roll_acc; // İvmeölçer verilerine dayalı olarak hesaplanan pitch ve roll açıları.
float angle_pitch, angle_roll; // Filtrelenmiş son pitch ve roll açıları.
int16_t raw_temp; // Ham sıcaklık verisini saklamak için kullanılan 16 bitlik tam sayı.
float temp; // İşlenmiş sıcaklık değerini saklamak için kullanılan kayan noktalı sayı.
int i; // Genel amaçlı döngü sayacı veya indeks değişkeni.
float prevtime, prevtime1, time1, elapsedtime1, prevtime2, time2, elapsedtime2; // Zaman ölçümü ve geçen süre hesaplamaları için kullanılan kayan noktalı sayılar.
HAL_StatusTypeDef set_gyro; // HAL fonksiyonlarının dönüş değerini saklamak için kullanılan enum türü.
// PID kontrolcüsü parametrelerini başlat
float Kp = 0.01f; // Oransal kazanç.
float Ki = 0.00000001f; // İntegral kazanç.
float Kd = 15.00f; // Türevsel kazanç.
// Hedef açıyı belirle (örneğin derece cinsinden)
float hedef_aci = 30.0f; // Sistem tarafından ulaşılması gereken hedef açı.
float mevcut_aci = 0.0f; // Sistemin mevcut açısı.
// Hata ve zaman için başlangıç değerlerini ayarla
float pid_cikti; // PID kontrolcüsünün çıktı değeri.
float hata = 0.0f; // Hedef açı ile mevcut açı arasındaki fark.
float turevsel_hata; // Hatanın zaman içindeki değişim oranı.
float onceki_hata = 0.0f; // Önceki hata değeri.
float toplam_hata = 0.0f; // Hataların zaman içindeki toplamı (integrali).
uint32_t onceki_zaman = 0; // Önceki zaman ölçümü.
uint32_t simdiki_zaman = 0; // �?imdiki zaman ölçümü.
float CCR_degeri; // PWM sinyalinin karşılaştırma değeri.
int sayim = 0; // Genel amaçlı sayaç veya durum bayrağı.
bool motorAktif = false; // Motorun aktif olup olmadığını kontrol eden değişken.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Mikrosaniye cinsinden zamanı döndüren fonksiyon
// STM32'de bu işlevi SysTick timer sağlar.
// SysTick timer kullanarak milisaniye cinsinden zamanı döndüren fonksiyon
uint32_t getMillis(){
    return HAL_GetTick();
}

// CCR değerini güncellemek için fonksiyon
void CCR_guncelle() {
    static unsigned long onceki_zaman = 0; // Önceki zamanı saklamak için statik değişken
    unsigned long simdiki_zaman = getMillis(); // Geçerli zamanı al
    unsigned long delta_zaman = simdiki_zaman - onceki_zaman; // Zaman farkını hesapla

    // Eğer sayım 0 ise, CCR değerini başlangıç değeri olan 50.0f yap ve sayımı 1 yap
     if (sayim == 0) {
         CCR_degeri = 50.0f;
         sayim = 1;
         onceki_zaman = simdiki_zaman;
     }
     // Eğer zaman farkı 100ms'den büyükse CCR değerini güncelle
    if (delta_zaman > 300) {
        // PID çıktısının işaretine göre CCR değerini artır veya azalt
        CCR_degeri += pid_cikti;
        onceki_zaman = simdiki_zaman; // Zamanı güncelle

        // CCR değerini minimum ve maksimum sınırlar içinde tut
        const float CCR_MIN = 50.0f;
        const float CCR_MAX = 100.0f;
        CCR_degeri = (CCR_degeri < CCR_MIN) ? CCR_MIN : (CCR_degeri > CCR_MAX) ? CCR_MAX : CCR_degeri;

        // CCR değerini PWM sinyali için güncelle
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CCR_degeri);
    }
}

// MPU6050 'den açı değerini oku
void mpu6050() {
    // Zaman ölçümü için değişkenler
    prevtime1 = time1; // Önceki zamanı sakla
    time1 = HAL_GetTick(); // Geçerli zamanı al
    elapsedtime1 = (time1 - prevtime1) * 1000; // Geçen süreyi hesapla

    // İvmeölçer verilerini oku
    tuffer[0] = 0x3B; // İvmeölçer veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tuffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, tuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

    // Ham ivmeölçer verilerini birleştir
    acc_raw[0] = (tuffer[0] << 8 | tuffer[1]); // X ekseni
    acc_raw[1] = (tuffer[2] << 8 | tuffer[3]); // Y ekseni
    acc_raw[2] = (tuffer[4] << 8 | tuffer[5]); // Z ekseni

    // Sıcaklık verisini oku
    buffer[0] = 0x41; // Sıcaklık veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buffer, 2, HAL_MAX_DELAY); // 2 bayt veri al

    // Ham sıcaklık verisini birleştir ve gerçek sıcaklığa çevir
    raw_temp = (buffer[0] << 8 | buffer[1]);
    temp = (raw_temp / 340.0) + 36.53; // MPU6050 datasheet formülüne göre hesaplama

    // Jiroskop verilerini oku
    cuffer[0] = 0x43; // Jiroskop veri başlangıç adresi
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY); // Adresi gönder
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

    // Ham jiroskop verilerini birleştir ve kalibrasyon değerlerini çıkar
    gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]) - gyro_cal[0]; // X ekseni
    gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]) - gyro_cal[1]; // Y ekseni
    gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]) - gyro_cal[2]; // Z ekseni

    // Jiroskop verilerini kullanarak açıları güncelle
    angle_pitch_gyro += gyro_raw[0] * 0.0000611; // Pitch açısı
    angle_roll_gyro += gyro_raw[1] * 0.0000611; // Roll açısı

    // Jiroskop verilerini kullanarak birbirine etki eden açıları güncelle
    angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
    angle_roll_gyro += angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);

    // İvmeölçer verilerinin toplam vektörünü hesapla
    acc_total_vector = sqrt((acc_raw[0] * acc_raw[0]) + (acc_raw[1] * acc_raw[1]) + (acc_raw[2] * acc_raw[2]));

    // İvmeölçer verilerini kullanarak açıları hesapla
    angle_pitch_acc = asin((float)acc_raw[1] / acc_total_vector) * 57.296; // Pitch açısı
    angle_roll_acc = asin((float)acc_raw[0] / acc_total_vector) * -57.296; // Roll açısı

    // Açıları kalibre et (gerekirse)
    angle_pitch_acc -= 0.00;
    angle_roll_acc -= 0.00;

    // Jiroskop verilerini ivmeölçer verileriyle birleştir
    if (set_gyro) {
        angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004; // Pitch açısı
        angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004; // Roll açısı
    } else {
        angle_pitch = angle_pitch_acc; // İlk okumada sadece ivmeölçer verilerini kullan
        set_gyro = true;
    }

    // Belirli bir süre beklemek için döngü
    while ((HAL_GetTick() - prevtime) * 1000 < 4000);
    prevtime = getMillis(); // Zamanı güncelle
}

// PID kontrol döngüsü
// Bu fonksiyon bir timer interrupt içinde çağrılmalıdır.
void pid_kontrol_dongusu() {

	 // Delta zamanı hesapla
	 uint32_t simdiki_zaman = getMillis();
	 uint32_t delta_zaman = simdiki_zaman - onceki_zaman;

    // Açı değerini oku
    mevcut_aci = fabs(angle_pitch);
    printf("Mevcut aci: %f\n", mevcut_aci); // @suppress("Float formatting support")

    // Hata hesapla
    hata = hedef_aci - mevcut_aci;

    // İntegral hata hesapla
    toplam_hata = toplam_hata + (hata * delta_zaman);

    // Türevsel hata hesapla
    turevsel_hata = (hata - onceki_hata) / delta_zaman;

    // PID çıktısını hesapla
    pid_cikti = (Kp * hata) + (Ki * toplam_hata) + (Kd * turevsel_hata);

    // Önceki hatayı ve zamanı güncelle
    onceki_hata = hata;
    onceki_zaman = simdiki_zaman;

    printf("Mevcut aci: %f\t", mevcut_aci); // @suppress("Float formatting support")
    printf("PID cikti: %f\n", pid_cikti); // @suppress("Float formatting support")
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

// MPU6050 Konfigürasyonu

  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
  // Sensörü uyandır ve iç saatini kullanmak üzere ayarla

  data = 0x08;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
  // Jiroskop için hassasiyeti ayarla (±500 derece/saniye)

  data = 0x10;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
  // İvmeölçer için hassasiyeti ayarla (±8g)

  // Jiroskop kalibrasyonu için 2000 ölçüm yap
  for (i = 0; i < 2000; i++) {
      prevtime2 = time2; // Önceki zamanı sakla
      time2 = HAL_GetTick(); // Geçerli zamanı al
      elapsedtime2 = (time2 - prevtime2) * 1000; // Geçen süreyi hesapla

      cuffer[0] = 0x43; // Jiroskop veri başlangıç adresi
      HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY); // Adresi gönder
      HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY); // 6 bayt veri al

      // Ham jiroskop verilerini birleştir
      gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]); // X ekseni
      gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]); // Y ekseni
      gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]); // Z ekseni

      // Kalibrasyon değerlerini topla
      gyro_cal[0] += gyro_raw[0];
      gyro_cal[1] += gyro_raw[1];
      gyro_cal[2] += gyro_raw[2];

      HAL_Delay(3); // 3 milisaniye bekle
  }

  // Kalibrasyon değerlerinin ortalamasını al
  gyro_cal[0] /= 2000;
  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;

  // Kalibrasyon işlemi başladığında LED'i yakın
  HAL_GPIO_WritePin(LED_PIN_MPU6050_GPIO_Port, LED_PIN_MPU6050_Pin,GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_PIN_MPU6050_GPIO_Port, LED_PIN_MPU6050_Pin,GPIO_PIN_RESET);
  HAL_Delay(500);



 // ESC başlangıcı ve kalibrasyonu
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PWM sinyalini başlat

  #if Calibrate

      TIM1->CCR1 = 100; // CCR1'i 100'e ayarla (genellikle maksimum gaz anlamına gelir)
      HAL_Delay(2000); // 2 saniye bekle (ESC'nin maksimum gazı algılaması için)
      TIM1->CCR1 = 50; // CCR1'i 50'ye ayarla (genellikle minimum gaz anlamına gelir)
      HAL_Delay(1000); // 1 saniye bekle (ESC'nin minimum gazı algılaması için)
      TIM1->CCR1 = 0; // CCR1'i 0'a ayarla (ESC'nin kalibrasyonunu tamamlaması için)
      // Kalibrasyon işlemi tamamlandığında LED'i söndür
      HAL_GPIO_WritePin(LED_PIN_ESC_GPIO_Port, LED_PIN_ESC_Pin,GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(LED_PIN_ESC_GPIO_Port, LED_PIN_ESC_Pin,GPIO_PIN_RESET);
      HAL_Delay(500);

  #endif



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Buton kontrolü
	  if(HAL_GPIO_ReadPin(BUTON_GPIO_Port, BUTON_Pin) == GPIO_PIN_SET) // Butona basıldı mı kontrol et
	  {
	    motorAktif = !motorAktif; // Motor aktiflik durumunu değiştir
	    if(motorAktif)
	    {
	      // Motor aktif olduğunda LED'i yak
	      HAL_GPIO_WritePin(LED_PIN_MOTOR_AKTIF_GPIO_Port, LED_PIN_MOTOR_AKTIF_Pin, GPIO_PIN_SET);
	    }
	    else
	    {
	      // Motor pasif olduğunda LED'i söndür
	      HAL_GPIO_WritePin(LED_PIN_MOTOR_AKTIF_GPIO_Port, LED_PIN_MOTOR_AKTIF_Pin, GPIO_PIN_RESET);
	    }
	    HAL_Delay(500); // Buton debouncing için bekleme süresi
	  }

	  // Motor aktif ise işlemleri yap
	  if(motorAktif)
	  {
	    // Motor aktifken yapılacak işlemler
	    mpu6050(); // MPU6050 sensöründen açı ve sıcaklık verilerini okur ve işler.
	    pid_kontrol_dongusu(); // PID kontrol döngüsünü çalıştırır, hedef açıya ulaşmak için gerekli düzeltmeleri hesaplar.
	    CCR_guncelle(); // PWM sinyalinin karşılaştırma değerini (CCR), PID kontrol çıktısına göre günceller.
	  }
	  else
	  {
	    // Motor pasif ise CCR_degeri'ni yavaşça 50'ye çek
	    while(CCR_degeri > 50.0f)
	    {
	      CCR_degeri -= 0.5; // CCR_degeri'ni 0.5 birim azalt
	      HAL_Delay(100); // 0.25 saniye bekle
	      TIM1->CCR1 = CCR_degeri; // Güncellenen CCR_degeri'ni CCR1'e ata
	    }

	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 420-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUTON_Pin|LED_PIN_MOTOR_AKTIF_Pin|LED_PIN_ESC_Pin|LED_PIN_MPU6050_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTON_Pin LED_PIN_MOTOR_AKTIF_Pin LED_PIN_ESC_Pin LED_PIN_MPU6050_Pin */
  GPIO_InitStruct.Pin = BUTON_Pin|LED_PIN_MOTOR_AKTIF_Pin|LED_PIN_ESC_Pin|LED_PIN_MPU6050_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
