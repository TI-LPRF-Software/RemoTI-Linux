#ifndef __OEM_SPI_SEMAPHORE_H_
#define __OEM_SPI_SEMAPHORE_H_

// OEMS SHOULD CHANGE THIS HEADER AND RE-DEFINE THESE MACROS TO CALL THEIR OWN APIs.

// This header provides MACROS to invoke an OEM-supplied set of APIs to share
// the SPI bus with other devices that may reside in the platform.  All of the
// APIs are presumed to return bool 'true' if success, 'false' if failed, and
// the default macros (used when the OEM doesn't need such a semaphore)
// simply always evaluate to 'true'.
//
// An example of an OEM implementation may be to create a system-wide semaphore
// that could be shared across processes, minimizing coupling of code between
// this code and the other device, but that choice is up to the OEM.


// Create SPI semaphore that is shared with logic for all SPI bus devices
#define OEM_SPI_CreateSharedSem()  true
// Take the shared semaphore (waiting if necessary) to lock the SPI bus
#define OEM_SPI_TakeSharedSem()    true
// TRY to take the shared semaphore (no waiting). If success returns true, otherwise false
#define OEM_SPI_TryTakeSharedSem() true
// Return the shared semaphore, thus unlocking the bus for other devices to use
#define OEM_SPI_ReturnSharedSem()  true

// OEM CUSTOMIZATION GOES HERE:

# endif //__OEM_SPI_SEMAPHORE_H_

