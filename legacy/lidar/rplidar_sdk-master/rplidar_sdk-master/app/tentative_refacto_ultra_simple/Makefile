#/*
# * Copyright (C) 2014  RoboPeak
# * Copyright (C) 2014 - 2018 Shanghai Slamtec Co., Ltd.
# *
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# */
#
HOME_TREE := ../../

MODULE_NAME := $(notdir $(CURDIR))

include $(HOME_TREE)/mak_def.inc

CXXSRC += main.cpp ImuOTOS.cpp lidar_lib.cpp
C_INCLUDES += -I$(CURDIR)/../../sdk/include -I$(CURDIR)/../../sdk/src
C_INCLUDES += -I$(CURDIR)

# Permet d'ajouter/surcharger les dossiers include du SDK si la lib/headers
# sont installés ailleurs (ex: SDK_INC_DIRS="/opt/slamtec/include /opt/slamtec/src")
ifdef SDK_INC_DIRS
C_INCLUDES += $(addprefix -I,$(SDK_INC_DIRS))
endif

EXTRA_OBJ := 
LD_LIBS += -lstdc++ -lpthread -lm

all: build_app

include $(HOME_TREE)/mak_common.inc

clean: clean_app

# ============================================================================
# CIBLES REFACTORÉES + Gestion lib SDK externe
# ============================================================================

# Variables pour la version refactorée
REFACTORED_TARGET = ultra_simple_refactored
REFACTORED_OBJS = main_refactored.o lidar_lib_refactored.o ImuOTOS.o

# SDK paths
SDK_DIR := $(CURDIR)/../../sdk

# Liste des dossiers candidats pour une lib déjà compilée (surchargeable)
SDK_LIB_SEARCH_DIRS ?= $(BUILD_OUTPUT_ROOT) \
	$(CURDIR)/../../sdk/lib \
	$(CURDIR)/../../lib \
	/usr/local/lib \
	/usr/lib

# Nom attendu de la lib statique
SDK_STATIC_NAME := libsl_lidar_sdk.a

# Si l'utilisateur donne directement le chemin complet
ifdef SDK_LIB_FILE
  SDK_LIB := $(SDK_LIB_FILE)
else
  # Cherche la première occurrence existante
  SDK_LIB := $(firstword $(wildcard $(addsuffix /$(SDK_STATIC_NAME),$(SDK_LIB_SEARCH_DIRS))))
endif

# Si toujours vide -> on prévoit de construire à partir des sources du dépôt
ifeq ($(SDK_LIB),)
  SDK_LIB := $(BUILD_OUTPUT_ROOT)/$(SDK_STATIC_MODULE)
  NEED_BUILD_SDK := 1
endif

ifeq ($(NEED_BUILD_SDK),1)
$(SDK_LIB):
	@echo "🧱 Lib SDK introuvable -> construction: $(SDK_LIB)"
	$(MAKE) -C $(SDK_DIR) build_sdk
else
$(SDK_LIB):
	@echo "➡️ Utilisation de la lib SDK précompilée: $(SDK_LIB)"
endif

# Règle principale pour la version refactorée (linke contre la lib SDK)
$(REFACTORED_TARGET): $(REFACTORED_OBJS) $(SDK_LIB)
	@echo "🔗 Linking $(REFACTORED_TARGET) avec $(SDK_LIB)..."
	$(CXX) -o $@ $(REFACTORED_OBJS) $(SDK_LIB) $(LDFLAGS)
	@echo "✅ Build réussi: $(REFACTORED_TARGET)"

# Compilation de main_refactored.cpp
main_refactored.o: main_refactored.cpp robot_config.h lidar_lib_refactored.h ImuOTOS.h
	@echo "🔨 Compilation main_refactored.cpp..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compilation de lidar_lib_refactored.cpp
lidar_lib_refactored.o: lidar_lib_refactored.cpp robot_config.h lidar_lib_refactored.h ImuOTOS.h
	@echo "🔨 Compilation lidar_lib_refactored.cpp..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Cibles utilitaires
.PHONY: clean-refactored
clean-refactored:
	@echo "🧹 Nettoyage version refactorée..."
	rm -f $(REFACTORED_OBJS) $(REFACTORED_TARGET)

.PHONY: all-versions
all-versions: $(APP_TARGET) $(REFACTORED_TARGET)
	@echo "✅ Versions originale et refactorée compilées"
