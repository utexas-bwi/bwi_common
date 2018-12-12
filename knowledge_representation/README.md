# knowledge_representation

Mechanisms for storing information about the world, and for querying this information.

## Installation

### Postgresql

    sudo apt install postgresql-server-dev-all


### Semi-automatic

Try running these commands through, one at a time. Leave the root password blank

    cd /tmp
    wget https://dev.mysql.com/get/mysql-apt-config_0.8.10-1_all.deb -O sql.deb
    sudo dpkg -i sql.deb
    sudo apt update
    sudo apt install -y mysql-server mysql-shell
    git clone https://github.com/mysql/mysql-connector-cpp.git
    cd mysql-connector-cpp
    mkdir build
    cd build
    cmake ..
    sudo make -j4 install
    sudo mv libmysqlcppconn* /usr/lib/x86_64-linux-gnu/
    sudo mysql -e "ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY ''"
    sudo mysql -u root -p -e "INSTALL PLUGIN mysqlx SONAME 'mysqlx.so';"
    roscd
    cd ../src/bwi_common/knowledge_representation
    mysql -u root -p -e "source sql/create_database.sql"
    
### Manual

If the above steps don't work, try these:

1. Make sure that MySQL is installed. Don't use your distribution's package. Get the latest `.deb` from [the MySQL website](https://dev.mysql.com/doc/mysql-apt-repo-quick-guide/en/#apt-repo-fresh-install
).
2. Next [clone MySQL Connector/C++ Version 8](https://dev.mysql.com/downloads/connector/cpp/) with 

    git clone https://github.com/mysql/mysql-connector-cpp.git

Then build:

    mkdir bulid
    cd build
    cmake ..
    sudo make -j2 install

  
3. You will also need MySQL Shell installed. Directions [here](https://dev.mysql.com/doc/refman/5.7/en/installing-mysql-shell-linux-quick.html).


4. Get the x plugin running with [these directions](https://dev.mysql.com/doc/refman/5.7/en/document-store-setting-up.html).

5. Run the `create_database.sql` file in MySQL using 

    mysql -u root -p -e "source sql/create_database.sql"
    
This will create the schema on your SQL server.
    
  
6. Build the catkin package.

## Development

We access the MySQL backing store via the xdev API. See the [documentation](https://dev.mysql.com/doc/dev/connector-cpp/8.0/) for the official MySQL xdev API C++ library.
