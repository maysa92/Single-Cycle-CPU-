        mov     1       [0+five]        load reg1 with 5 (uses symbolic address)
        mov     2       [0+two]         load reg2 with 3 
        mov     3       [0+six]         load reg3 with 5        
start   cmp     1       2
        je      start                   goto start when reg1==reg2
        cmp     3       1
        je      start                   goto start when reg1==reg3
done    halt                            end of program
five    dd      5
two     dd      3
six     dd      5
stAddr  dd      start                   will contain the address of start (2)