git clone https://chromium.googlesource.com/libyuv/libyuv
cd libyuv
git reset --hard 4a14cb2e81235ecd656e799aecaaf139db8ce4a2
cmake .
make install
cd ..

curl -O https://capnproto.org/capnproto-c++-0.6.1.tar.gz
tar xvf capnproto-c++-0.6.1.tar.gz
cd capnproto-c++-0.6.1
./configure --prefix=/usr/local CPPFLAGS=-DPIC CFLAGS=-fPIC CXXFLAGS=-fPIC LDFLAGS=-fPIC --disable-shared --enable-static
make -j12
make install
cd ..

git clone https://github.com/commaai/c-capnproto.git
cd c-capnproto
git submodule update --init --recursive
autoreconf -f -i -s
CFLAGS="-fPIC" ./configure --prefix=/usr/local
make -j12
make install
cd ..

curl -LO https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz
tar xfz zeromq-4.2.3.tar.gz
cd zeromq-4.2.3
./autogen.sh
./configure CPPFLAGS=-DPIC CFLAGS=-fPIC CXXFLAGS=-fPIC LDFLAGS=-fPIC --disable-shared --enable-static
make -j12
make install
cd ..

git clone git://github.com/zeromq/czmq.git
cd czmq
./autogen.sh
./configure CPPFLAGS=-DPIC CFLAGS=-fPIC CXXFLAGS=-fPIC LDFLAGS=-fPIC --disable-shared --enable-static
make check -j12
make install
ldconfig
cd ..

git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
cp zmq.hpp /usr/local/include/zmq.hpp
cp zmq_addon.hpp /usr/local/include/zmq_addon.hpp
cd /tmp/

bash
