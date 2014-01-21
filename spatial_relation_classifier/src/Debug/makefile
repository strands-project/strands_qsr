################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utils/impl/subdir.mk
-include TestCases/TestCase1/impl/subdir.mk
-include FeatureExtractionModule/SingleObjectFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/SceneSingleObjectFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/SceneObjectPairFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/ObjectPairFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/DatabaseSingleObjectFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/DatabaseObjectPairFeatureModule/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSingleObject/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneSingleObject/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneObjectPair/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionObjectPair/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseSingleObject/impl/subdir.mk
-include FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseObjectPair/impl/subdir.mk
-include DatabaseHandlingModule/SceneInformationModule/impl/subdir.mk
-include DatabaseHandlingModule/ObjectInformationModule/impl/subdir.mk
-include DatabaseHandlingModule/DatabaseInformationModule/impl/subdir.mk
-include DatabaseHandlingModule/ApisConvertionModule/ApiConvertSimulationDBModule/impl/subdir.mk
-include DatabaseHandlingModule/ApisConvertionModule/ApiConvertKTHDBModule/impl/subdir.mk
-include ApisStatisticalToolModule/Training/impl/subdir.mk
-include ApisStatisticalToolModule/Test/impl/subdir.mk
-include ApisStatisticalToolModule/StoreTraining/impl/subdir.mk
-include ApisStatisticalToolModule/StatisticalToolModule/impl/subdir.mk
-include ApisStatisticalToolModule/ArrangeFeatureTraining/impl/subdir.mk
-include ApisStatisticalToolModule/ArrangeFeatureTestScene/impl/subdir.mk
-include ApiDatabaseFrequencies/impl/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Scene_Object_Classification

# Tool invocations
Scene_Object_Classification: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -L/opt/ros/groovy/lib -o "Scene_Object_Classification" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) Scene_Object_Classification
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
