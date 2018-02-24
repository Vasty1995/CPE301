/*
 * AssemblerApplication3.asm
 *
 *  Created: 2/24/2018 1:25:21 PM
 *   Author: kengneta
 */ 


 



	.org 0
	ldi R21, 0			
	ldi XL, 0x22		;X to 0x0222
	ldi XH, 0x02
	ldi YL, 0x00		;Y to 0x0400
	ldi YH, 0x04
	ldi ZL, 0x00		;Z to 0x0600
	ldi ZH, 0x06

FILL:
	ldi R20, 0
	add R20, XL	
	add R20, XH			
	st X, R20			
	inc XL				
	cpi XL, 255			
	brsh RESETX			
CONTINUE:
	inc R21				 
	jmp DIVIDE			
DIVIDERET:
	cpi R21, 255		
	brne FILL			
	ldi R21, 0			
FILLMORE:
	ldi R20, 0
	add R20, XL			
	add R20, XH
	st X, R20			
	inc XL				
	inc R21				
	jmp DIVIDE2			
DIVIDERET2:
	cpi R21, 42			
	brne FILLMORE
	jmp END				

RESETX:
	ldi XL, 0			
	inc XH				
	jmp CONTINUE

DIVIDE:
	ldi R22, 5			
	ldi R23, 0			
	mov R24, R20		
L1:
	inc R23				
	sub R20, R22		
	BRCC L1				
	dec R23				
	add R20, R22		
	cpi R20, 0			
	brne NOTDIV		
	st Y, R24			
	inc YL				
	ldi R25, 0	
	add R16, R24		
	adc R17, R15		
	jmp DIVIDERET

NOTDIV:
	st Z, R24			
	inc ZL				
	ldi R25, 0
	add R18, R24		
	adc R19, R15		
	jmp DIVIDERET

	DIVIDE2:
	ldi R22, 5			
	ldi R23, 0
	mov R24, R20		
L2:
	inc R23				
	sub R20, R22		
	BRCC L2				
	dec R23				
	add R20, R22	
	cpi R20, 0			
	brne NOTDIV2		 
	st Y, R24			 
	inc YL				
	ldi R25, 0
	add R16, R24		
	adc R17, R25		
	jmp DIVIDERET2

NOTDIV2:
	st Z, R24			
	inc ZL				
	ldi R25, 0
	add R18, R24		
	add R19, R25		
	jmp DIVIDERET2

END:
	jmp END				