#include <avr/io.h>
#include <string.h>

#include "serial.h"

#include "SDL_timer.h"

// Windows Header Files:
#include <windows.h>
#include <commctrl.h>

static DCB PortDCB;
static COMMTIMEOUTS CommTimeouts;
static HANDLE hPort1;

serialSim::serialSim()
{
    UCSR0A.setCallback(DELEGATE(registerDelegate, serialSim, *this, UART_UCSR0A_callback));
    UDR0.setCallback(DELEGATE(registerDelegate, serialSim, *this, UART_UDR0_callback));

    UCSR0A = 0;

    recvLine = 0;
    recvPos = 0;
    memset(recvBuffer, '\0', sizeof(recvBuffer));

    //-----------------------------------------------------------

    hPort1 = CreateFile (TEXT("COM3"),			            // Name of the port
                GENERIC_READ | GENERIC_WRITE,     // Access (read-write) mode
                0,
                NULL,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                NULL);

    if ( hPort1 != INVALID_HANDLE_VALUE ) {

        //Initialize the DCBlength member.
        PortDCB.DCBlength = sizeof (DCB);

        // Get the default port setting information.
        GetCommState (hPort1, &PortDCB);
        configure();

        // Retrieve the time-out parameters for all read and write operations
        GetCommTimeouts (hPort1, &CommTimeouts);
        configuretimeout();

        //Re-configure the port with the new DCB structure.
        if (!SetCommState (hPort1, &PortDCB) || !SetCommTimeouts (hPort1, &CommTimeouts) || !PurgeComm(hPort1, PURGE_TXCLEAR | PURGE_RXCLEAR))
        {
            CloseHandle(hPort1);
        }
    }
}

void serialSim::configure()
{
    // Change the DCB structure settings
    PortDCB.fBinary = TRUE;                         // Binary mode; no EOF check
    PortDCB.fParity = TRUE;                         // Enable parity checking
    PortDCB.fDsrSensitivity = FALSE;                // DSR sensitivity
    PortDCB.fErrorChar = FALSE;                     // Disable error replacement
    PortDCB.fOutxDsrFlow = FALSE;                   // No DSR output flow control
    PortDCB.fAbortOnError = FALSE;                  // Do not abort reads/writes on error
    PortDCB.fNull = FALSE;                          // Disable null stripping
    PortDCB.fTXContinueOnXoff = TRUE;                // XOFF continues Tx

    PortDCB.BaudRate= 115200;
    PortDCB.ByteSize = 8;
    PortDCB.Parity = NOPARITY;
    PortDCB.StopBits =  ONESTOPBIT;
    PortDCB.fOutxCtsFlow = FALSE;                      // No CTS output flow control
    PortDCB.fDtrControl = DTR_CONTROL_ENABLE;          // DTR flow control type
    PortDCB.fOutX = FALSE;                             // No XON/XOFF out flow control
    PortDCB.fInX = FALSE;                              // No XON/XOFF in flow control
    PortDCB.fRtsControl = RTS_CONTROL_ENABLE;          // RTS flow control
}

void serialSim::configuretimeout()
{
	CommTimeouts.ReadIntervalTimeout = 1;
	CommTimeouts.ReadTotalTimeoutConstant = 1;
	CommTimeouts.ReadTotalTimeoutMultiplier = 1;
	CommTimeouts.WriteTotalTimeoutMultiplier = 10;
	CommTimeouts.WriteTotalTimeoutConstant = 50;
}

serialSim::~serialSim()
{
    if ( hPort1 != INVALID_HANDLE_VALUE )
        CloseHandle(hPort1);
}

void serialSim::UART_UCSR0A_callback(uint8_t oldValue, uint8_t& newValue)
{
    //Always mark "write ready" flag, so the serial code never waits.
    newValue |= _BV(UDRE0);
}

static unsigned int ticks = 0;

struct ring_buffer
{
  unsigned char buffer[/* RX_BUFFER_SIZE */ 128];
  int head;
  int tail;
};

extern ring_buffer rx_buffer;

void serialSim::poll()
{
    uint8_t newValue;

    BOOL ret;
    DWORD dwRead;
    OVERLAPPED osReader = {0};
    unsigned long retlen=0;

    unsigned int _ticks = SDL_GetTicks();
    if (ticks + 5 > _ticks)
        return;

    ticks = _ticks;

    if ( hPort1 != INVALID_HANDLE_VALUE )
    {
        osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (osReader.hEvent)
        {

            if (ReadFile(hPort1, &newValue, 1, &dwRead,  &osReader) && dwRead)
            {
                fprintf(stderr, "%c", newValue);

                // UDR0.forceValue(newValue);
                // UCSR0A.forceValue( UCSR0A | _BV(RXC0));

                // MSerial.checkRx();

                int i = (unsigned int)(rx_buffer.head + 1) % /* RX_BUFFER_SIZE */ 128;

                // if we should be storing the received character into the location
                // just before the tail (meaning that the head would advance to the
                // current location of the tail), we're about to overflow the buffer
                // and so we don't write the character or advance the head.
                if (i != rx_buffer.tail) {
                  rx_buffer.buffer[rx_buffer.head] = newValue;
                  rx_buffer.head = i;
                }
            }

            CloseHandle(osReader.hEvent);
        }
    }
}

void serialSim::UART_UDR0_callback(uint8_t oldValue, uint8_t& newValue)
{
    recvBuffer[recvLine][recvPos] = newValue;
    recvPos++;
    if (recvPos == 80 || newValue == '\n')
    {
        recvPos = 0;
        recvLine++;
        if (recvLine == SERIAL_LINE_COUNT)
        {
            for(unsigned int n=0; n<SERIAL_LINE_COUNT-1;n++)
                memcpy(recvBuffer[n], recvBuffer[n+1], 80);
            recvLine--;
            memset(recvBuffer[recvLine], '\0', 80);
        }
    }

    //-----------------------------------------------------------
    if ( hPort1 != INVALID_HANDLE_VALUE ) {
        DWORD dwNumBytesWritten;
        WriteFile (hPort1, &newValue, 1, &dwNumBytesWritten, NULL);
    }
}

void serialSim::draw(int x, int y)
{
    for(unsigned int n=0; n<SERIAL_LINE_COUNT;n++)
        drawStringSmall(x, y+n*3, recvBuffer[n], 0xFFFFFF);
}
