..\x264 ^
	--input-res 864x464 ^
	--frames 3  ^
	--keyint 301 ^
	--bitrate 10000 ^
	--psnr ^
	--ssim ^
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
	--no-8x8dct ^
	--partition none ^
	--subme 0 ^
	--weightp 0 ^
test.yuv ^
-o 3frames.264 ^
&1>3frames_print.log