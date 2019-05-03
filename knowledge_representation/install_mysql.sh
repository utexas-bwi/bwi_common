#!/usr/bin/env bash
cd /tmp
wget https://dev.mysql.com/get/mysql-apt-config_0.8.12-1_all.deb -O sql.deb
sudo dpkg -i sql.deb
sudo apt update
sudo apt install -y mysql-server mysql-shell
git clone https://github.com/mysql/mysql-connector-cpp.git
cd mysql-connector-cpp
git checkout tags/8.0.15
mkdir build
cd build
cmake ..
sudo make -j4 install
sudo mv libmysqlcppconn* /usr/lib/x86_64-linux-gnu/

