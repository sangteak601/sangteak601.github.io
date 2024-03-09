---
title:  "github.io 블로그 시작하기"
excerpt: "GitHub Blog 서비스인 github.io 블로그 시작하기로 했다."
toc: True
toc_label: "목차"
toc_sticky: True
categories:
  - Github
tags:
  - Blog
last_modified_at: 2021-07-16
---

이 글의 제목은 {{ page.title }}이고
마지막으로 수정된 시간은 {{ page.last_modified_at }}이다.

# 정렬
왼쪽 텍스트 정렬
{: .text-left}

중앙 텍스트 정렬
{: .text-center}

오른쪽 텍스트 정렬
{: .text-right}

# 이모지
https://en.wikibooks.org/wiki/Unicode/List_of_useful_symbols

# 이미지
이미지 중앙
![image-center](/assets/images/filename.jpg){: .align-center}

이미지 왼쪽
![image-left](/assets/images/filename.jpg){: .align-left}

이미지 오른쪽
![image-right](/assets/images/filename.jpg){: .align-right}

풀 사이즈 이미지
![full](/assets/images/filename.jpg)
{: .full}

![styled-image](/assets/images/pixel_tracker_logo_80px.png "This is some hover text"){: .align-center style="width: 5%;"}

![styled-image](/assets/images/pixel_tracker_logo_80px.png "This is some hover text"){: .align-center style="width: 10%;"}
Some custom styled caption with a [_link_](#via-markdown).
{: .align-caption}

[![styled-image](/assets/images/pixel_tracker_logo_80px.png "This is some hover text"){: .align-center style="width: 10%;"}](/assets/images/pixel_tracker_logo_80px.png "Title shown in gallery view")
Some custom styled caption with a [_link_](#via-markdown).
{: .align-caption}

# 노티스 박스
Default
{: .notice}

Primary
{: .notice--primary}

Info
{: .notice--info}

Warning
{: .notice--warning}

Success
{: .notice--success}

Danger
{: .notice--danger}

# Code block 하이라이트 지원 언어 목록
Language	Usage in Jekyll	Aliases
ActionScript	actionscript	as, as3
configuration files for Apache web server	apache
Markdown based API description language.	apiblueprint	apiblueprint, apib
AppleScript scripting language by Apple Inc. (http://developer.apple.com/applescript/)	applescript	applescript
BIML (Business Intelligence Markup Language)	biml
The C programming language	c
Say more, more clearly.	ceylon
CFScript (the CFML scripting language)	cfscript	cfc
The Clojure programming language (clojure.org)	clojure	clj, cljs
The cross-platform, open-source build system	cmake
The Coffeescript programming language (coffeescript.org)	coffeescript	coffee, coffee-script
The Common Lisp variant of Lisp (common-lisp.net)	common_lisp	cl, common-lisp, elisp, emacs-lisp
A generic lexer for configuration files	conf	config, configuration
Coq (coq.inria.fr)	coq
The C++ programming language	cpp	c++
a multi-paradigm language targeting .NET	csharp	c#, cs
Cascading Style Sheets, used to style web pages	css
The D programming language(dlang.org)	d	dlang
The Dart programming language (dart.dev)	dart
Lexes unified diffs or patches	diff	patch, udiff
Eiffel programming language	eiffel
Elixir language (elixir-lang.org)	elixir	elixir, exs
Embedded ruby template files	erb	eruby, rhtml
The Erlang programming language (erlang.org)	erlang	erl
Factor, the practical stack language (factorcode.org)	factor
Fortran 95 Programming Language	fortran
A business-readable spec DSL (github.com/cucumber/cucumber/wiki/Gherkin)	gherkin	cucumber, behat
The GLSL shader language	glsl
The Go programming language (http://golang.org)	go	golang
A powerful build system for the JVM	gradle
The Groovy programming language (http://www.groovy-lang.org/)	groovy
The Haml templating system for Ruby (haml.info)	haml	HAML
the Handlebars and Mustache templating languages	handlebars	hbs, mustache
The Haskell programming language (haskell.org)	haskell	hs
HTML, the markup language of the web	html
http requests and responses	http
the INI configuration format	ini
The IO programming language (http://iolanguage.com)	io
The Java programming language (java.com)	java
JavaScript, the browser scripting language	javascript	js
Django/Jinja template engine (jinja.pocoo.org)	jinja	django
JavaScript Object Notation with extenstions for documentation	json-doc
JavaScript Object Notation (json.org)	json
An elegant, formally-specified config language for JSON	jsonnet
The Julia programming language	julia	jl
Kotlin (http://kotlinlang.org)	kotlin
Liquid is a templating engine for Ruby (liquidmarkup.org)	liquid
Literate coffeescript	literate_coffeescript	litcoffee
Literate haskell	literate_haskell	lithaskell, lhaskell, lhs
The LLVM Compiler Infrastructure (http://llvm.org/)	llvm
Lua (http://www.lua.org)	lua
Makefile syntax	make	makefile, mf, gnumake, bsdmake
Markdown, a light-weight markup language for authors	markdown	md, mkd
Matlab	matlab	m
Moonscript (http://www.moonscript.org)	moonscript	moon
Netwide Assembler	nasm
configuration files for the nginx web server (nginx.org)	nginx
The Nim programming language (http://nim-lang.org/)	nim	nimrod
an extension of C commonly used to write Apple software	objective_c	objc
Objective CAML (ocaml.org)	ocaml
a procedural programming language commonly used as a teaching language.	pascal
The Perl scripting language (perl.org)	perl	pl
The PHP scripting language (php.net)	php	php, php3, php4, php5
A boring lexer that doesn’t highlight anything	plaintext	text
powershell	powershell	posh
The Praat scripting language (praat.org)	praat
The Prolog programming language (http://en.wikipedia.org/wiki/Prolog)	prolog
.properties config files for Java	properties
Google’s language-neutral, platform-neutral, extensible mechanism for serializing structured data	protobuf	proto
The Puppet configuration management language (puppetlabs.org)	puppet	pp
The Python programming language (python.org)	python	py
QML, a UI markup language	qml
The R statistics language (r-project.org)	r	R, s, S
Racket is a Lisp descended from Scheme (racket-lang.org)	racket
The Ruby programming language (ruby-lang.org)	ruby	rb
The Rust programming language (rust-lang.org)	rust	rs
The Sass stylesheet language language (sass-lang.com)	sass
The Scala programming language (scala-lang.org)	scala	scala
The Scheme variant of Lisp	scheme
SCSS stylesheets (sass-lang.com)	scss
sed, the ultimate stream editor	sed
A generic lexer for shell session and command line	shell_session	terminal, console
Various shell languages, including sh and bash	shell	bash, zsh, ksh, sh
The Slim template language	slim
The Smalltalk programming language	smalltalk	st, squeak
Smarty Template Engine	smarty
Standard ML	sml	ml
Structured Query Language, for relational databases	sql
Multi paradigm, compiled programming language developed by Apple for iOS and OS X development. (developer.apple.com/swift)	swift
Test Anything Protocol	tap
The Tool Command Language (tcl.tk)	tcl
The TeX typesetting system	tex	TeX, LaTeX, latex
the TOML configuration format (https://github.com/mojombo/toml)	toml
The tulip programming language (http://github.com/jneen/tulip)	tulip	tlp
Twig template engine (twig.symfony.com)	twig
TypeScript, a superset of JavaScript	typescript	ts
Visual Basic	vb	visualbasic
The System Verilog hardware description language	verilog
VimL, the scripting language for the Vim editor (vim.org)	viml	vim, vimscript, ex
XML	xml
Yaml Ain’t Markup Language (yaml.org)	yaml	yml