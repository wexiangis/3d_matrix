# 编译器选择
CC = gcc

# ----- 文件夹列表 -----

DIR_COMMON = common
DIR_3D = 3d
DIR_UI = ui
# obj目录用于缓存.o文件
DIR_OBJ = obj

# ----- 文件夹编译及.o文件转储 -----

# 头文件目录列表
INC = -I$(DIR_3D) -I$(DIR_UI) -I$(DIR_COMMON)

%.o:../$(DIR_COMMON)/%.c
	$(CC) -O3 -Wall -c $< $(INC) -o $@
%.o:../$(DIR_3D)/%.c
	$(CC) -O3 -Wall -c $< $(INC) -o $@
%.o:../$(DIR_UI)/%.c
	$(CC) -O3 -Wall -c $< $(INC) -o $@

# ----- obj中的.o文件统计 -----

obj-common = ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_COMMON)/*.c}}}
obj-3d = ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_3D)/*.c}}}
obj-ui = ${patsubst %.c,$(DIR_OBJ)/%.o,${notdir ${wildcard $(DIR_UI)/*.c}}}

#----- 把所有.o文件链接,最终编译 -----

out: $(obj-3d) $(obj-ui) $(obj-common)
	$(CC) -Wall -o out main.c $(INC)

clean:
	@rm ./obj/* out
