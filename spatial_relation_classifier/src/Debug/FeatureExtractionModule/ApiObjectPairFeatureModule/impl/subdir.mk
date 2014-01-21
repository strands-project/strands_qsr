################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../FeatureExtractionModule/ApiObjectPairFeatureModule/impl/ApiObjectPairFeature.cpp 

OBJS += \
./FeatureExtractionModule/ApiObjectPairFeatureModule/impl/ApiObjectPairFeature.o 

CPP_DEPS += \
./FeatureExtractionModule/ApiObjectPairFeatureModule/impl/ApiObjectPairFeature.d 


# Each subdirectory must supply rules for building sources it contributes
FeatureExtractionModule/ApiObjectPairFeatureModule/impl/%.o: ../FeatureExtractionModule/ApiObjectPairFeatureModule/impl/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/groovy/include/pcl-1.6/pcl/common/ -I/opt/ros/groovy/include/pcl-1.6/pcl/common/impl/ -I/opt/ros/groovy/include/tf_conversions/ -I/opt/ros/groovy/include/pcl-1.6/ -I/opt/ros/groovy/inlclude/ -I/opt/ros/groovy/include/opencv2/core/ -I/opt/ros/groovy/include/opencv2/ -I/opt/ros/groovy/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


