# Memory Efficiency
Compiled with `-Os` flag as this was needed to integrate Tensorflow Lite Micro.
## None (App disabled)
```
Flash |  239276/1032192 (23%),  792916 free | text: 233476, data: 5800, ccmdata: 0
RAM   |   71216/131072  (54%),   59856 free | bss: 65416, data: 5800
CCM   |   58524/65536   (89%),    7012 free | ccmbss: 58524, ccmdata: 0
```
## None (App enabled)
```
Flash |  239240/1032192 (23%),  792952 free | text: 233448, data: 5792, ccmdata: 0
RAM   |   75304/131072  (57%),   55768 free | bss: 69512, data: 5792
CCM   |   58620/65536   (89%),    6916 free | ccmbss: 58620, ccmdata: 0
```
## Filter Algorithm 
```
Flash |  240456/1032192 (23%),  791736 free | text: 234656, data: 5800, ccmdata: 0
RAM   |   75328/131072  (57%),   55744 free | bss: 69528, data: 5800
CCM   |   58716/65536   (90%),    6820 free | ccmbss: 58716, ccmdata: 0
```
## TFLM Algorithm
```
Flash |  437244/1032192 (42%),  594948 free | text: 428620, data: 8624, ccmdata: 0
RAM   |   90552/131072  (69%),   40520 free | bss: 81928, data: 8624
CCM   |   58716/65536   (90%),    6820 free | ccmbss: 58716, ccmdata: 0
```
## Filter + TFLM Algorithm
```
Flash |  437932/1032192 (42%),  594260 free | text: 429308, data: 8624, ccmdata: 0
RAM   |   90552/131072  (69%),   40520 free | bss: 81928, data: 8624
CCM   |   58716/65536   (90%),    6820 free | ccmbss: 58716, ccmdata: 0
```