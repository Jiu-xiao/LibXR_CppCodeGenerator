#!/usr/bin/env python
import argparse
import fnmatch
import os
import re
import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def modify_interrupt_file(file_path):
    """Modify interrupt handler files to add UART Rx callback functions."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.readlines()

    extern_declaration = """#ifdef HAL_UART_MODULE_ENABLED
extern void STM32_UART_ISR_Handler_IDLE(UART_HandleTypeDef *huart);
#endif"""
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
    pattern = re.compile(r'void\s+(USART\d+|UART\d+)_IRQHandler\s*\(\s*void\s*\)', re.IGNORECASE)

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
    modified_functions = []

    if modified:
        with open(file_path, "w", encoding="utf-8") as f:
            f.writelines(content)
        logging.info(f"Modified {file_path}: Inserted callbacks for {modified_functions}")
    else:
        logging.info(f"No changes needed in {file_path}.")

    return modified, modified_functions

def main():
    parser = argparse.ArgumentParser(description="Modify STM32 interrupt handler files.")
    parser.add_argument("input_dir", type=str, help="Directory containing *_it.c files")

    args = parser.parse_args()
    input_directory = args.input_dir

    if not os.path.isdir(input_directory):
        logging.error(f"Input directory does not exist: {input_directory}")
        exit(1)

    total_modified_files = 0
    total_modified_functions = []

    for filename in os.listdir(input_directory):
        if fnmatch.fnmatch(filename, "*_it.c"):
            file_path = os.path.join(input_directory, filename)
            modified, modified_funcs = modify_interrupt_file(file_path)
            if modified:
                total_modified_files += 1
                total_modified_functions.extend(modified_funcs)

    logging.info(f"Summary: Modified {total_modified_files} files.")
    if total_modified_functions:
        logging.info(f"Modified interrupt handlers: {', '.join(total_modified_functions)}")
    else:
        logging.info("No interrupt handlers needed changes.")
