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
ifeq "$(wildcard nbproject/Makefile-local-dspic33ep512mu810.mk)" "nbproject/Makefile-local-dspic33ep512mu810.mk"
include nbproject/Makefile-local-dspic33ep512mu810.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=dspic33ep512mu810
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/traps.c ../src/system_config/exp16/dspic33ep512mu810/main.c ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/60018555/main.o ${OBJECTDIR}/_ext/60018555/i2c_emem.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/traps.o.d ${OBJECTDIR}/_ext/60018555/main.o.d ${OBJECTDIR}/_ext/60018555/i2c_emem.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/60018555/main.o ${OBJECTDIR}/_ext/60018555/i2c_emem.o

# Source Files
SOURCEFILES=../src/traps.c ../src/system_config/exp16/dspic33ep512mu810/main.c ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-dspic33ep512mu810.mk dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.ok ${OBJECTDIR}/_ext/1360937237/traps.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d" -o ${OBJECTDIR}/_ext/1360937237/traps.o ../src/traps.c    
	
${OBJECTDIR}/_ext/60018555/main.o: ../src/system_config/exp16/dspic33ep512mu810/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/60018555 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o.ok ${OBJECTDIR}/_ext/60018555/main.o.err 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60018555/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/60018555/main.o.d" -o ${OBJECTDIR}/_ext/60018555/main.o ../src/system_config/exp16/dspic33ep512mu810/main.c    
	
${OBJECTDIR}/_ext/60018555/i2c_emem.o: ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/60018555 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o.d 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o.ok ${OBJECTDIR}/_ext/60018555/i2c_emem.o.err 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60018555/i2c_emem.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/60018555/i2c_emem.o.d" -o ${OBJECTDIR}/_ext/60018555/i2c_emem.o ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c    
	
else
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.ok ${OBJECTDIR}/_ext/1360937237/traps.o.err 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d" -o ${OBJECTDIR}/_ext/1360937237/traps.o ../src/traps.c    
	
${OBJECTDIR}/_ext/60018555/main.o: ../src/system_config/exp16/dspic33ep512mu810/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/60018555 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o.ok ${OBJECTDIR}/_ext/60018555/main.o.err 
	@${RM} ${OBJECTDIR}/_ext/60018555/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60018555/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/60018555/main.o.d" -o ${OBJECTDIR}/_ext/60018555/main.o ../src/system_config/exp16/dspic33ep512mu810/main.c    
	
${OBJECTDIR}/_ext/60018555/i2c_emem.o: ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/60018555 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o.d 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o.ok ${OBJECTDIR}/_ext/60018555/i2c_emem.o.err 
	@${RM} ${OBJECTDIR}/_ext/60018555/i2c_emem.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60018555/i2c_emem.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I"../src" -O1 -MMD -MF "${OBJECTDIR}/_ext/60018555/i2c_emem.o.d" -o ${OBJECTDIR}/_ext/60018555/i2c_emem.o ../src/system_config/exp16/dspic33ep512mu810/i2c_emem.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--stack=16,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--stack=16,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/iic_eeprom.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/dspic33ep512mu810
	${RM} -r dist/dspic33ep512mu810

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
