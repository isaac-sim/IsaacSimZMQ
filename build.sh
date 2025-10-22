#!/bin/bash
set -e

SCRIPT_DIR=$(dirname "${BASH_SOURCE}")

# First we must fetch dependencies, to be able to compile protobuf
bash "$SCRIPT_DIR/repo.sh" build --fetch-only || exit $?

# Remove old build binaries
rm -rf exts/isaacsim.zmq.bridge/bin
rm -rf exts/isaacsim.zmq.bridge.examples/pip_prebundle

# Compile protobuf
pushd proto

./../_build/target-deps/protobuf/bin/protoc --proto_path=. --cpp_out=. --python_out=. client_stream_message.proto
./../_build/target-deps/protobuf/bin/protoc --proto_path=. --python_out=. server_control_message.proto

cp client_stream_message.pb.h ../exts/isaacsim.zmq.bridge/plugins/nodes/client_stream_message.pb.h
cp client_stream_message.pb.cc ../exts/isaacsim.zmq.bridge/plugins/nodes/client_stream_message.pb.cc
cp client_stream_message_pb2.py ../isaac-zmq-server/src/client_stream_message_pb2.py
cp client_stream_message_pb2.py ../exts/isaacsim.zmq.bridge.examples/isaacsim/zmq/bridge/examples/core/client_stream_message_pb2.py
cp server_control_message_pb2.py ../isaac-zmq-server/src/server_control_message_pb2.py
cp server_control_message_pb2.py ../exts/isaacsim.zmq.bridge.examples/isaacsim/zmq/bridge/examples/core/server_control_message_pb2.py

rm client_stream_message.pb.h
rm client_stream_message.pb.cc
rm client_stream_message_pb2.py
rm server_control_message_pb2.py

popd

# Build
bash "$SCRIPT_DIR/repo.sh" build -x -r || exit $?

# Copy build artifacts into exts path for simplicity

# isaacsim.zmq.bridge.examples
cp -r _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge.examples/PACKAGE-LICENSES exts/isaacsim.zmq.bridge.examples/PACKAGE-LICENSES
cp -rL _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge.examples/pip_prebundle exts/isaacsim.zmq.bridge.examples/pip_prebundle

# isaacsim.zmq.bridge
cp -r _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge/bin exts/isaacsim.zmq.bridge/bin
cp -r _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge/ogn exts/isaacsim.zmq.bridge/ogn
cp -r _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge/isaacsim/zmq/bridge/ogn exts/isaacsim.zmq.bridge/isaacsim/zmq/bridge/ogn
cp -r _build/linux-$(uname -m)/release/exts/isaacsim.zmq.bridge/PACKAGE-LICENSES exts/isaacsim.zmq.bridge/PACKAGE-LICENSES


# Copy shared libraries from target-deps
mkdir -p exts/isaacsim.zmq.bridge/bin/lib
# cp -L _build/target-deps/zmq/lib/libsodium.so.18 exts/isaacsim.zmq.bridge/bin/lib/libsodium.so.23
# cp -L _build/target-deps/zmq/lib/libsodium.so.18 exts/isaacsim.zmq.bridge/bin/lib/libsodium.so.18
# cp -L _build/target-deps/zmq/lib/libpgm-5.2.so.0 exts/isaacsim.zmq.bridge/bin/lib/libpgm-5.2.so.0
# cp -L _build/target-deps/zmq/lib/libnorm.so.1 exts/isaacsim.zmq.bridge/bin/lib/libnorm.so.1
cp -L _build/target-deps/zmq/lib/libzmq.so exts/isaacsim.zmq.bridge/bin/lib/libzmq.so
cp -L _build/target-deps/zmq/lib/libzmq.so.5 exts/isaacsim.zmq.bridge/bin/lib/libzmq.so.5
cp -L _build/target-deps/zmq/lib/libzmq.so.5 exts/isaacsim.zmq.bridge/bin/lib/libzmq.so.5.2.5


# Clean up build directories
rm -rf _conan _repo _compiler _deps
