cd c:\Program Files (x86)\Arduino\hardware\tools\avr\bin\
avrdude.exe -p atmega328p -c avrispv2 -P COM7 -U hfuse:w:0xD2:m -C ../etc/avrdude.conf