#!/usr/bin/env python
import argparse
import fnmatch
import os
import re


def modify_interrupt_file(file_path):
    """Modify interrupt handler files to add UART Rx callback functions."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.readlines()

    extern_declaration = "extern void STM32_UART_ISR_Handler_IDLE(UART_HandleTypeDef *huart);"
    callback_template = "  STM32_UART_ISR_Handler_IDLE(&huart{});\n"
    modified = False

    # Check if extern declaration exists
    if extern_declaration not in "".join(content):
        for i, line in enumerate(content):
            if "/* USER CODE BEGIN 0 */" in line:
                content.insert(i + 1, extern_declaration + "\n")
                modified = True
                break

    # Match USART/UART IRQ handlers
    pattern = re.compile(r'void\s+(USART\d+|UART\d+)_IRQHandler\s*\(\s*void\s*\)')
    for i, line in enumerate(content):
        match = pattern.search(line)
        if match:
            usart = match.group(1)
            uart_number = re.search(r'\d+', usart).group()  # Extract UART number

            # Find USER CODE BEGIN block
            user_code_begin_pattern = re.compile(rf'/\*\s*USER CODE BEGIN {usart}_IRQn 1\s*\*/')
            for j in range(i, len(content)):
                if user_code_begin_pattern.search(content[j]):
                    # Check if callback already exists
                    callback_call = callback_template.format(uart_number).strip()
                    if callback_call not in "".join(content[j:j + 5]):
                        # Insert callback after USER CODE BEGIN
                        content.insert(j + 1, callback_template.format(uart_number))
                        modified = True
                    break

    # Write back only if modified
    if modified:
        with open(file_path, "w", encoding="utf-8") as f:
            f.writelines(content)
        print(f"[Pass] Modified {file_path}: Added HAL_UART_RxCpltCallback calls.")
    else:
        print(f"[Pass] {file_path}: No changes needed.")


def main():
    parser = argparse.ArgumentParser(description="Modify STM32 interrupt handler files.")
    parser.add_argument("input_dir", type=str, help="Directory containing *_it.c files")

    args = parser.parse_args()
    input_directory = args.input_dir

    for filename in os.listdir(input_directory):
        if fnmatch.fnmatch(filename, "*_it.c"):
            file_path = os.path.join(input_directory, filename)
            modify_interrupt_file(file_path)
