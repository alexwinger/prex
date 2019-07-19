
SRCS:=		arm/arch/locore.S \
		arm/arch/cpufunc.S \
		arm/arch/cpu.c \
		arm/arch/trap.c \
		arm/arch/context.c \
		arm/skrzg1m/interrupt.c \
		arm/skrzg1m/clock.c \
		arm/skrzg1m/machdep.c

ifeq ($(CONFIG_MMU),y)
SRCS+=		arm/arch/mmu.c
endif
ifeq ($(DEBUG),1)
SRCS+=		arm/skrzg1m/diag.c
endif
