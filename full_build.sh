if make -f STM32Make.make; then
  if STM32_Programmer_CLI -c port=/dev/ttyUSB0 -d build/debug/Battery_Heating.elf; then
    echo "Succeeded"
    clear
    serust -p /dev/ttyUSB0
  else
    echo "Command failed, trying again"
    if STM32_Programmer_CLI -c port=/dev/ttyUSB0 -d build/debug/Battery_Heating.elf; then
      clear
      serust -p /dev/ttyUSB0
    fi
  fi
fi
