# 定义编译器和编译选项
CXX = g++
CXXFLAGS = -Iinclude -lstage

# 定义源文件和目标文件
SRC = main.cpp
OBJ = build/main.o
TARGET = build/main

# 默认目标
all: $(TARGET) symlinks

# 编译目标文件
$(OBJ): $(SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 链接生成可执行文件
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(CXXFLAGS)

# 创建符号链接
symlinks:
	ln -sf ../*.png build/
	ln -sf ../*.world build/

# 清理生成的文件
clean:
	rm -f build/*.o build/main

# 运行生成的可执行文件
run: $(TARGET)
	cd build && ./main


.PHONY: all clean symlinks