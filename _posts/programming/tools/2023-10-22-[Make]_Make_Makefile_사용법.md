---
title:  "[Make] Make Makefile 사용법"
excerpt: "컴파일 언어로 프로그램을 작성하게 되면 인터프리터 언어와는 달리 반드시 컴파일 과정을 거쳐야지만 실행 가능한 파일을 생성할 수 있다. Make는 이러한 컴파일 과정을 좀 더 쉽고 빠르게 할 수 있도록 도와주는 도구이다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - Tools
last_modified_at: 2023-10-22
---

**&#9432; Note!** 이 포스트는 Linux 운영체제를 기반으로 작성되었습니다.
{: .notice--info}

## Make 란?

컴파일 언어로 프로그램을 작성하게 되면 인터프리터 언어와는 달리 반드시 컴파일 과정을 거쳐야지만 실행 가능한 파일을 생성할 수 있다. Make는 이러한 컴파일 과정을 좀 더 쉽고 빠르게 할 수 있도록 도와주는 도구이다. Make는 기본적으로 Makefile 이라고 하는 파일로 부터 필요한 정보를 얻기 때문에 Make 툴을 사용하기 위해서는 Makefile을 필요에 맞게 작성하는 것이 필요하다.

## 기초 사용법
Makefile의 기본적인 구조는 다음과 같다.

```make
target: prerequisites
<TAB> recipe
<TAB> recipe
```

여기서 `target`은 일반적으로 생성하고자 하는 파일 이름이고, `prerequisites`은 target을 생성하기 위해 필요한 파일의 목록이다. 마지막으로 `recipe`는 파일을 생성하기 위한 명령어이다.

실제 예시를 작성하여 실행해보자. 빈 폴더를 생성 후, 폴더 내에 `Makefile` 을 생성한 뒤, 아래 내용을 작성한다.

```make
hello:
  echo "Hello World"
```

생성한 폴더로 이동 후, `make` 명령어를 실행하면 다음과 같은 출력을 확인할 수 있다.

```
echo "Hello World"
Hello World
```

기본적으로 make 명령어를 실행하게 되면 Makefile에 있는 첫번째 `target`을 생성하기 위한 명령어만 실행하게 된다. 이를 확인하기 위해 아래와 같이 `Makefile`을 수정한 뒤, `make` 명령어를 실행해보자.

```make
hello:
  echo "Hello World"

clean:
  echo "Good bye"
```

여전히 같은 출력을 확인할 수 있을 것이다. 이렇게 첫번째 `target`이 아닌 다른 `target`을 실행하고 싶다면 `target` 이름을 `make` 명령어 뒤에 붙여주면 된다. 아래 명령어를 실행해보자.

```shell
$ make clean
```

성공적으로 실행되었다면 아래의 결과를 확인할 수 있을 것이다.

```
echo "Good bye"
Good bye
```

출력된 결과를 보면 실행하고자하는 명령어를 먼저 출력하고, 그 결과를 출력하는 것을 확인할 수 있다. 명령어를 출력하고 싶지 않다면 `@echo "Hello World` 와 같이 해당 명령어 앞에 `@`를 붙어주면 된다.


## C 파일 예시 및 Prerequisites 활용법

앞서 살펴본 기초 사용법을 토대로 실제 C 파일을 컴파일 해보자. 먼저, 아래와 같은 내용의 `hello_world.c` 파일을 같은 폴더 내에 생성한다.

```c
#include <stdio.h>

int main()
{
  printf("Hello World\n");
  return 0;
}
```

`Makefile`을 아래와 같이 수정한다.

```make
hello_world: hello_world.o
	gcc hello_world.o -o hello_world

hello_world.o: hello_world.c
	gcc -c hello_world.c -o hello_world.o
```

`make` 명령어를 실행한 뒤, 컴파일된 파일을 실행해보자.

```
$ make
$ ./hello_world
```

정상적으로 실행되었다면 `Hello World` 라는 문구를 출력했을 것이다. 이제 `Makefile`의 내용을 이해해보자.

먼저, 앞서 언급했던 것처럼 별도로 `target`을 지정해주지 않으면 항상 첫번째 `target`을 실행하게 되어 있다. 여기서는 `hello_world`가 첫번째 `target`이다. 그런데 `hello_world` 는 `hello_world.o` 를 prerequisite 으로 명시하고 있다. 따라서, `hello_world` 에 해당하는 명령어를 바로 실행하는 것이 아니라, `hello_world.o` 파일이 존재하는지를 먼저 확인하게 된다. 해당 파일이 존재한다면 바로 명령어를 실행하겠지만, 그렇지 않다면 `Makefile` 내에 해당 파일을 생성하기 위한 명령어가 있는지 확인한다. 이 경우에는 `hello_world.o`이 존재하므로 해당 명령어를 먼저 수행하게 된다.

성공적으로 컴파일을 한 뒤에, 한 번 더 `make` 명령어를 실행하게 되면 `make: 'hello_world' is up to date.` 라는 내용을 출력하면서 다시 컴파일을 하지는 않는다. 이는 이미 해당 파일이 존재하고, 컴파일을 한 뒤에 내용이 수정되지 않았기 때문이다. 이를 확인하기 위해 다음과 같이 `hello_world.c` 파일을 수정한 뒤, 다시 `make` 명령어를 실행해보자.

```c
#include <stdio.h>

int main()
{
  printf("Hello World\n");
  printf("Good bye\n");
  return 0;
}
```

이번에는 파일이 다시 컴파일되는 것을 확인할 수 있을 것이다.

이처럼 prerequisites에 정의된 `target` 을 먼저 생성하기 때문에, 이를 이용해서 원하는 명령어를 실행할 수 있다. 아래의 예시를 한 번 살펴보자.

```make
all: something1 something2 something3

something1:
  @echo "something1"

something2:
  @echo "something2"

something3:
  @echo "something3"
```

첫번째 `target`이 `all` 이므로, `all` 을 생성하기 위한 명령어를 실행할 것이다. 다만, `all` 을 생성하기 위해 먼저 3가지 다른 파일이 존재해야 하므로, 이들을 생성하기 위한 명령어를 먼저 실행할 것이다. 따라서, 이 경우 3가지 다른 명령어를 모두 실행하게 된다. 이처럼 prerequisites을 이용하여 실행하고자 하는 명령어를 한 번의 실행으로 모두 실행할 수 있다.

여기서 주의할 점은 prerequisites은 파일의 존재 유무를 확인한다는 것이다. 따라서 만약 아래와 같이 파일을 생성한 뒤에 앞서 생성한 `Makefile` 을 실행하게 되면 원하는 결과를 얻지 못할 것이다.

```shell
$ touch something1 something2 something3
```

파일을 모두 생성한 뒤, 다시 `make` 명령어를 실행하게 되면 `make: Nothing to be done for 'all'.` 와 같은 내용을 출력하며 어떠한 행동도 하지 않는다. 이는 `all` 을 생성하기 위한  명령어를 정의하지 않았기 때문이다. 때로는 이처럼 `target` 파일을 생성하지 않는 경우도 있다. 이 경우에 해당 파일의 존재 유무가 명령어의 실행여부를 결정하는 것을 원치 않을 것이다. 이를 해결하기 위해 `.PHONY` 라고 하는 `target`을 정의할 수 있다.

```make
.PHONY: something1 something2 something3
all: something1 something2 something3

something1:
  @echo "something1"

something2:
  @echo "something2"

something3:
  @echo "something3"
```

위와 같이 `Makefile`을 수정한 뒤, 다시 `make` 명령어를 실행하게 되면 다시 3가지를 모두 실행하는 것을 확인할 수 있다. `.PHONY`에 정의된 `target`은 파일의 존재유무와 관계없이 명령어를 실행하게 된다.

## 변수 사용법

`Makefile` 을 좀 더 동적으로 작성하기 위해 변수를 활용할 수 있다. 먼저 변수는 아래와 같이 정의하고 불러올 수 있다.

```make
var := myvariable

print:
  @echo "variable:" $(var)
```

`make` 명령어를 실행하면 `variable: myvariable` 와 같은 출력을 확인할 수 있다.

특정한 내용을 담고 있는 변수도 있다. 아래의 예시를 보자.

```make
something: something1 something2 something3
  @echo "target name:" $@
  @echo "changed prerequisites:" $?
  @echo "all prerequisites:" $^
  touch something

something1:
  touch something1

something2:
  touch something2

something3:
  touch something3
```

`make` 명령어를 실행하면 아래와 같은 결과를 확인할 수 있다.

```
target name: something
changed prerequisites: something1 something2 something3
all prerequisites: something1 something2 something3
touch something
```

이후 아래와 같이 `something1` 의 수정시간을 변경한 뒤, 다시 실행하게 되면 결과가 달라지는 것을 확인할 수 있다.

```shell
$ touch something1
$ make
```

결과는 다음과 같다.

```
target name: something
changed prerequisites: something1
all prerequisites: something1 something2 something3
touch something
```

또다른 유용한 변수로 Wildcard가 있다. 이는 다음과 같이 특정 확장자를 가진 모든 파일을 불러오기 용이하다.

```make
all_c_files := $(wildcard *.c)
```

## 간단한 템플릿
앞서 확인한 내용을 바탕으로 간단한 `Makefile` 템플릿을 작성해보자.

```make
CC := gcc
LIBS := -lm
SRCS := $(wildcard src/*.c)
OBJS := obj/$(basename $(notdir $(SRCS))).o
BINS := bin/$(basename $(notdir $(SRCS)))

.PHONY: all clean
all: $(BINS)

$(BINS): $(OBJS)
        @echo "Create executable:" $@
        $(CC) -o $@ $(OBJS) $(LIBS)

obj/%.o: src/%.c
        @echo "Create object:" $@
        $(CC) -c $^ -o $@

clean:
        @echo "Cleaning"
        @rm bin/* obj/*

```