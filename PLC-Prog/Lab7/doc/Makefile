.PHONY: all build clean

all: build build_short clean

build:
	pdflatex Lab7.tex
	pdflatex Lab7.tex
	pdflatex Lab7.tex

build_short:
	pdflatex Lab7_short.tex
	pdflatex Lab7_short.tex
	pdflatex Lab7_short.tex

clean:
	rm -f *.aux
	rm -f *.lof
	rm -f *.log
	rm -f *.lol
	rm -f *.lot
	rm -f *.out
	rm -f *.toc
