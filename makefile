all:
	gcc a.c -o asm
	./asm cmp.ja.asm cmp.ja.mc
	iverilog *.v && ./a.out
clean:
	rm asm cmp.ja.mc
