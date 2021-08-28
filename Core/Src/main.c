/**
  ******************************************************************************
  * @file    main.c
  * @author  Marco, Roldan L.
  * @version v1.0
  * @date    August 14, 2021
  * @brief   USART driver test code
  ******************************************************************************
  *
  * Copyright (C) 2021  Marco, Roldan L.
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.en.html.
  *
  *
  * https://github.com/rmarco30
  *
  ******************************************************************************
**/

#include "stm32f10x.h"
#include <stdint.h>
#include "usart1.h"

extern char usart1_rdata[10];

int main()
{
    usart1_init();

    while (1)
    {
        usart1_write("hello test\n");
        usart1_write("the quick brown fox jumps over the lazy dog\n");
    }
}