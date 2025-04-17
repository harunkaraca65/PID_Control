# PID_Control
PID Control Explanation for Beam Balancing (English)
In this project, a PID control algorithm was implemented on the STM32F407VG microcontroller to keep a beam balanced at a specific target angle. The system continuously reads the current angle of the beam using an IMU sensor. This angle is compared to the desired target angle to calculate the error.

The PID algorithm processes this error in three parts:

Proportional: Reacts to the current error.

Integral: Accumulates past errors to eliminate steady-state error.

Derivative: Predicts future behavior by analyzing the rate of change.

These three components are combined to calculate the final control output. This output is used to drive a motor, which adjusts the beam’s position to minimize the error.

The control loop runs continuously. In each loop, the program calculates the error, updates the integral and derivative terms, and then generates a new motor output. The system is tuned by adjusting the PID coefficients (Kp, Ki, Kd) to get a stable and responsive behavior.

Kiriş Dengeleme için PID Kontrol Açıklaması (Türkçe)
Bu projede, bir kirişi belirli bir hedef açıda sabit tutmak amacıyla STM32F407VG mikrodenetleyicisi üzerinde bir PID kontrol algoritması uygulanmıştır. Sistem, bir IMU sensörü ile sürekli olarak kirişin mevcut açısını okur. Bu açı, hedef açı ile karşılaştırılarak hata değeri hesaplanır.

PID algoritması bu hatayı üç ana bileşenle işler:

Oransal (P): Anlık hataya doğrudan tepki verir.

İntegral (I): Geçmişteki hataları biriktirerek kalıcı hatayı azaltır.

Türevsel (D): Hatanın değişim hızını analiz ederek sistemi önceden dengeler.

Bu üç bileşen bir araya getirilerek kontrol çıkışı hesaplanır. Bu çıkış, bir motoru sürmek için kullanılır ve motorun hareketi ile kirişin pozisyonu düzeltilerek hata azaltılır.

Kontrol döngüsü sürekli çalışır. Her döngüde, hata hesaplanır, integral ve türev terimleri güncellenir ve motor çıkışı yeniden belirlenir. Sistem, PID katsayıları (Kp, Ki, Kd) uygun şekilde ayarlanarak kararlı ve hızlı bir tepki verecek şekilde optimize edilmiştir.
