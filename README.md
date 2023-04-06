# MIoT-Mesh-Realtek

## Partition Table

|    Name  | Size |
|:--------:|:----:|
| Reserved |   4K |
| OEM configuration | 4K |
| OTA Header | 4K |
| Secure Boot | 4K |
| Patch1 | 40K |
| Application | 220K |
| FTL | 16K |
| OTA Buffer | 204K |
| Patch2 | 12K |
| Mijia Certification | 4K |

## OTA Buffer

|    Name  | Size |
|:--------:|:----:|
| Patch    | 40K  |
| Secure Boot | 4K |
| Header   | 1K   |
| Compressed Application | 159K |

## watchdog功能
所有demo中watchdog功能默认开启，通过修改工程文件夹下的otp_config.h中的宏ROM_WATCH_DOG_ENABLE为0来关闭

## hardfault记录功能
所有demo中hardfault记录功能默认关闭，通过修改工程文件夹下的otp_config.h中的宏ENABLE_WRITE_HARDFAULT_RECORD_TO_FLASH宏为1来开启# test1
