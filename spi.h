/* 
 * File:   spi.h
 * Author: sean
 *
 * Created on January 22, 2014, 3:27 PM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif

void spiSlaveDisable(char);
void spiSlaveEnable(char);
char readSPIByte(void);
int readTemperature(void);


#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

