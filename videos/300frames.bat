..\x264 ^
	--input-res 1152x528 ^
	--frames 300  ^
	--keyint 301 ^
	--fps 25.0 ^
 	^
	--bitrate 10000 ^
	--psnr ^
 	^
	--profile baseline ^
	--preset ultrafast ^
 	^
	--no-8x8dct ^
	--bframes 0 ^
	--no-cabac ^
	--no-deblock ^
	--no-mbtree ^
	--rc-lookahead 0 ^
	--ref 1 ^
	--no-mixed-refs ^
	--scenecut 0 ^
	--trellis 0 ^
	--no-weightb ^
	--partition none ^
	--subme 0 ^
	--weightp 0 ^
	--me dia ^
	--verbose ^
	--threads 1 ^
	--aq-mode 0 ^
test.yuv ^
-o 300frames.264 ^
&1>300frames_print.log
