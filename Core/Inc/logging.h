#ifndef LOGGING_H_
#define LOGGING_H_

// https://stackoverflow.com/questions/2670816/how-can-i-use-the-compile-time-constant-line-in-a-string/2671100
// https://gcc.gnu.org/onlinedocs/gcc-4.8.5/cpp/Stringification.html
// These two macros allow us to add __LINE__, __FILE__, etc to strings at compile-time
#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)

// https://stm32f4-discovery.net/2015/06/get-interrupt-execution-status-on-cortex-m-processors/
// ^ Might be useful someday to check if we're currently in an IRQ. Never print in an IRQ unless it's blocking

void LOG_printf(char *format, ...);
void LOG(char *message);

#endif /* LOGGING_H_ */

