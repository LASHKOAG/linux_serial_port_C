/*
	UART communication on Raspberry Pi using C (WiringPi Library)
	http://www.electronicwings.com
	
	Raspberry Pi UART Communication using Python and C
    https://www.electronicwings.com/raspberry-pi/raspberry-pi-uart-communication-using-python-and-c
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>


#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

// #include "wiringPi.h"                //the code was taken from this library for RaspberryPi
// #include "wiringSerial.h"

int serialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case      50: myBaud =      B50 ; break ;
    case      75: myBaud =      B75 ; break ;
    case     110: myBaud =     B110 ; break ;
    case     134: myBaud =     B134 ; break ;
    case     150: myBaud =     B150 ; break ;
    case     200: myBaud =     B200 ; break ;
    case     300: myBaud =     B300 ; break ;
    case     600: myBaud =     B600 ; break ;
    case    1200: myBaud =    B1200 ; break ;
    case    1800: myBaud =    B1800 ; break ;
    case    2400: myBaud =    B2400 ; break ;
    case    4800: myBaud =    B4800 ; break ;
    case    9600: myBaud =    B9600 ; break ;
    case   19200: myBaud =   B19200 ; break ;
    case   38400: myBaud =   B38400 ; break ;
    case   57600: myBaud =   B57600 ; break ;
    case  115200: myBaud =  B115200 ; break ;
    case  230400: myBaud =  B230400 ; break ;
    case  460800: myBaud =  B460800 ; break ;
    case  500000: myBaud =  B500000 ; break ;
    case  576000: myBaud =  B576000 ; break ;
    case  921600: myBaud =  B921600 ; break ;
    case 1000000: myBaud = B1000000 ; break ;
    case 1152000: myBaud = B1152000 ; break ;
    case 1500000: myBaud = B1500000 ; break ;
    case 2000000: myBaud = B2000000 ; break ;
    case 2500000: myBaud = B2500000 ; break ;
    case 3000000: myBaud = B3000000 ; break ;
    case 3500000: myBaud = B3500000 ; break ;
    case 4000000: myBaud = B4000000 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;  // Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;  // 10mS

  return fd ;
}




  /*
 * serialFlush:
 *  Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void serialFlush (const int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *  Release the serial port
 *********************************************************************************
 */

void serialClose (const int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *  Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar (const int fd, const unsigned char c)
{
  write (fd, &c, 1) ;
}


/*
 * serialPuts:
 *  Send a string to the serial port
 *********************************************************************************
 */

void serialPuts (const int fd, const char *s)
{
  write (fd, s, strlen (s)) ;
}

/*
 * serialPrintf:
 *  Printf over Serial
 *********************************************************************************
 */

void serialPrintf (const int fd, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataAvail:
 *  Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataAvail (const int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *  Get a single character from the serial device.
 *  Note: Zero is a valid character and this function will time-out after
 *  10 seconds.
 *********************************************************************************
 */

int serialGetchar (const int fd)
{
  uint8_t x ;

  if (read (fd, &x, 1) != 1)
    return -1 ;

  return ((int)x) & 0xFF ;
}




int main ()
{
  int serial_port ;
  char dat;
  // if ((serial_port = serialOpen ("/dev/ttyS0", 9600)) < 0)	/* open serial port */
  if ((serial_port = serialOpen ("/dev/ttyACM0", 9600)) < 0)  /* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }



  //if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
  //{
  //  fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  //  return 1 ;
  //}

  while(1){
	  
	if(serialDataAvail (serial_port) )
	{ 
		dat = serialGetchar (serial_port);		/* receive character serially*/	
		printf ("%c", dat) ;
		fflush (stdout) ;
		serialPutchar(serial_port, dat);		/* transmit character serially on port */
		  }
	}

return 0;
}