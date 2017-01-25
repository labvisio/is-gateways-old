Pioneer Robot Gateway
===================

Installing Dependencies

```bash
# Installing communication middleware
git clone https://github.com/labviros/is-cpp
cd is-cpp/scripts
chmod 755 install
./install

# Installing robot driver dependencies
wget http://robots.mobilerobots.com/ARIA/download/current/ARIA-src-2.9.1.tar.gz
tar -xf ARIA-src-2.9.1.tar.gz.1
cd Aria-src-2.9.1
make allLibs -j `nproc`
sudo cp lib/* /usr/local/lib
sudo mkdir /usr/local/include/Aria
sudo cp include/* /usr/local/include/Aria
```