#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-PIC24EP512GU810_PIM.mk)" "nbproject/Makefile-local-PIC24EP512GU810_PIM.mk"
include nbproject/Makefile-local-PIC24EP512GU810_PIM.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC24EP512GU810_PIM
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/exp16/pic24ep512gu810_pim/system.c ../src/usb_descriptors.c ../src/main_usb.c ../src/initialize.c ../src/usb_function.c ../../../../../../framework/usb/src/usb_device.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1682852123/system.o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ${OBJECTDIR}/_ext/1360937237/main_usb.o ${OBJECTDIR}/_ext/1360937237/initialize.o ${OBJECTDIR}/_ext/1360937237/usb_function.o ${OBJECTDIR}/_ext/838585624/usb_device.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1682852123/system.o.d ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d ${OBJECTDIR}/_ext/1360937237/main_usb.o.d ${OBJECTDIR}/_ext/1360937237/initialize.o.d ${OBJECTDIR}/_ext/1360937237/usb_function.o.d ${OBJECTDIR}/_ext/838585624/usb_device.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1682852123/system.o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ${OBJECTDIR}/_ext/1360937237/main_usb.o ${OBJECTDIR}/_ext/1360937237/initialize.o ${OBJECTDIR}/_ext/1360937237/usb_function.o ${OBJECTDIR}/_ext/838585624/usb_device.o

# Source Files
SOURCEFILES=../src/system_config/exp16/pic24ep512gu810_pim/system.c ../src/usb_descriptors.c ../src/main_usb.c ../src/initialize.c ../src/usb_function.c ../../../../../../framework/usb/src/usb_device.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-PIC24EP512GU810_PIM.mk dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24EP256GU810
MP_LINKER_FILE_OPTION=,-Tp24EP256GU810.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1682852123/system.o: ../src/system_config/exp16/pic24ep512gu810_pim/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1682852123" 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o.d 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o.ok ${OBJECTDIR}/_ext/1682852123/system.o.err 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1682852123/system.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1682852123/system.o.d" -o ${OBJECTDIR}/_ext/1682852123/system.o ../src/system_config/exp16/pic24ep512gu810_pim/system.c    
	
${OBJECTDIR}/_ext/1360937237/usb_descriptors.o: ../src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.ok ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ../src/usb_descriptors.c    
	
${OBJECTDIR}/_ext/1360937237/main_usb.o: ../src/main_usb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o.ok ${OBJECTDIR}/_ext/1360937237/main_usb.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main_usb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/main_usb.o.d" -o ${OBJECTDIR}/_ext/1360937237/main_usb.o ../src/main_usb.c    
	
${OBJECTDIR}/_ext/1360937237/initialize.o: ../src/initialize.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o.ok ${OBJECTDIR}/_ext/1360937237/initialize.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/initialize.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/initialize.o.d" -o ${OBJECTDIR}/_ext/1360937237/initialize.o ../src/initialize.c    
	
${OBJECTDIR}/_ext/1360937237/usb_function.o: ../src/usb_function.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o.ok ${OBJECTDIR}/_ext/1360937237/usb_function.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_function.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_function.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_function.o ../src/usb_function.c    
	
${OBJECTDIR}/_ext/838585624/usb_device.o: ../../../../../../framework/usb/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/838585624" 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o.ok ${OBJECTDIR}/_ext/838585624/usb_device.o.err 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/838585624/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/838585624/usb_device.o.d" -o ${OBJECTDIR}/_ext/838585624/usb_device.o ../../../../../../framework/usb/src/usb_device.c    
	
else
${OBJECTDIR}/_ext/1682852123/system.o: ../src/system_config/exp16/pic24ep512gu810_pim/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1682852123" 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o.d 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o.ok ${OBJECTDIR}/_ext/1682852123/system.o.err 
	@${RM} ${OBJECTDIR}/_ext/1682852123/system.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1682852123/system.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1682852123/system.o.d" -o ${OBJECTDIR}/_ext/1682852123/system.o ../src/system_config/exp16/pic24ep512gu810_pim/system.c    
	
${OBJECTDIR}/_ext/1360937237/usb_descriptors.o: ../src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.ok ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ../src/usb_descriptors.c    
	
${OBJECTDIR}/_ext/1360937237/main_usb.o: ../src/main_usb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o.ok ${OBJECTDIR}/_ext/1360937237/main_usb.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main_usb.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main_usb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/main_usb.o.d" -o ${OBJECTDIR}/_ext/1360937237/main_usb.o ../src/main_usb.c    
	
${OBJECTDIR}/_ext/1360937237/initialize.o: ../src/initialize.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o.ok ${OBJECTDIR}/_ext/1360937237/initialize.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/initialize.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/initialize.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/initialize.o.d" -o ${OBJECTDIR}/_ext/1360937237/initialize.o ../src/initialize.c    
	
${OBJECTDIR}/_ext/1360937237/usb_function.o: ../src/usb_function.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o.ok ${OBJECTDIR}/_ext/1360937237/usb_function.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_function.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_function.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_function.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_function.o ../src/usb_function.c    
	
${OBJECTDIR}/_ext/838585624/usb_device.o: ../../../../../../framework/usb/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/838585624" 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o.ok ${OBJECTDIR}/_ext/838585624/usb_device.o.err 
	@${RM} ${OBJECTDIR}/_ext/838585624/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/838585624/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -I"../../../../../../framework" -I"../../../../../../bsp/exp16/pic24ep512gu810_pim" -I"../src/system_config/exp16/pic24ep512gu810_pim" -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -MMD -MF "${OBJECTDIR}/_ext/838585624/usb_device.o.d" -o ${OBJECTDIR}/_ext/838585624/usb_device.o ../../../../../../framework/usb/src/usb_device.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../src/CMG_LIB.X.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}   ..\src\CMG_LIB.X.a      -Wl,--defsym=__MPLAB_BUILD=1,--no-pack-data,--no-handles,--no-isr,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../src/CMG_LIB.X.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}   ..\src\CMG_LIB.X.a      -Wl,--defsym=__MPLAB_BUILD=1,--no-pack-data,--no-handles,--no-isr,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/TheCellMusicGear.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PIC24EP512GU810_PIM
	${RM} -r dist/PIC24EP512GU810_PIM

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
