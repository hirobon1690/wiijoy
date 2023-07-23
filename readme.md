# wiimote_as_joy
## About
Wiiリモコン(RVL-CNT-01, RVL-CNT-01-TR)をROS2のjoyとして機能させるノード．

## Requirements
- [wiiuse](https://github.com/wiiuse/wiiuse)
```bash
git clone https://github.com/wiiuse/wiiuse.git
cd wiiuse
mkdir build
cd build
cmake .. [-DCMAKE_INSTALL_PREFIX=/usr/local] [-DCMAKE_BUILD_TYPE=Release] [-DBUILD_EXAMPLE_SDL=NO]
sudo make install
```

## Usage
### 導入
```bash
cd ~/ros2_ws/src
git clone https://github.com/hirobon1690/wiimote_as_joy.git
cd ../
colcon build
source install/setup.bash
```

### 実行
```bash
ros2 run wiimote_as_joy joy_node
```

## Configuration
`src/keymap.h`を書き換えてビルドする．
書き方は
```cpp
#define [Wiiのキー] [対応させたいXboxのキー]
```

## License
GNU GPL