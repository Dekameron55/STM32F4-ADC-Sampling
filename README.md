# STM32F4-ADC-Sampling
STM32F446 MCU Project involving FreeRTOS tasks to update LCD 16x2 display via I2C interface with ADC measurement data.

The project consists of 3 tasks with different priorities, 1 Single Shot timer and Mutex protection.
1. Display task - This task updates the LCD display with Number of button presses by the user.
2. GPIO task - This task check the GPIO status on GPIO Pin A5 (PA5, LED pin on Nucleo) and counts how many times the button has been pressed.
3. ADC task - This task measures the voltage drop on a potentiometer.

If the user pushes the blue button, the LED will light up. After that the Single Shot timer will be triggered, trough interrupt and callback function.
After the timer has expired, after 5 sec the LED will shut light off on it's own and increment an internal counter, which counts how many times the user has pressed the button. Durring this time the GPIO task will constantly monitor the GPIO state (Either 1 or 0) and send this information to the Display Task.

The ADC task will be sampling the data and will also update it on the Display.

The Display data function is mutex protected, so if a task enters the display task durring r/w operations of another task, the task trying to take the mutex will return back to waiting state.

The printf function is rerouted and mutex protected also by random access by different tasks. This function prints out the given states of a task and showcases the CPU switching between the different tasks.



![alt text](https://user-images.githubusercontent.com/61906443/204617961-89ce7803-6c8a-4040-bc60-ceeb2111294a.jpg)
