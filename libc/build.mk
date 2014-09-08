libc :=
musl =

ifeq ($(arch),x64)
musl_arch = x86_64
else
musl_arch = notsup
endif

libc += internal/_chk_fail.o
libc += internal/floatscan.o
libc += internal/intscan.o
libc += internal/libc.o
libc += internal/shgetc.o

musl += ctype/__ctype_get_mb_cur_max.o
musl += ctype/__ctype_tolower_loc.o
musl += ctype/__ctype_toupper_loc.o
libc += ctype/isalnum.o
musl += ctype/isalpha.o
libc += ctype/isascii.o
libc += ctype/isblank.o
libc += ctype/iscntrl.o
musl += ctype/isdigit.o
musl += ctype/isgraph.o
musl += ctype/islower.o
musl += ctype/isprint.o
libc += ctype/ispunct.o
libc += ctype/isspace.o
musl += ctype/isupper.o
libc += ctype/iswalnum.o
musl += ctype/iswalpha.o
libc += ctype/iswblank.o
libc += ctype/iswcntrl.o
musl += ctype/iswctype.o
musl += ctype/iswdigit.o
libc += ctype/iswgraph.o
libc += ctype/iswlower.o
libc += ctype/iswprint.o
libc += ctype/iswpunct.o
libc += ctype/iswspace.o
libc += ctype/iswupper.o
libc += ctype/iswxdigit.o
libc += ctype/isxdigit.o
libc += ctype/toascii.o
libc += ctype/tolower.o
libc += ctype/toupper.o
libc += ctype/towctrans.o
libc += ctype/wcswidth.o
musl += ctype/wctrans.o
libc += ctype/wcwidth.o

musl += dirent/alphasort.o
musl += dirent/scandir.o
libc += dirent/fdopendir.o

libc += env/__environ.o
musl += env/clearenv.o
libc += env/getenv.o
musl += env/putenv.o
musl += env/setenv.o
musl += env/unsetenv.o

musl += ctype/__ctype_b_loc.o

musl += errno/strerror.o

musl += locale/catclose.o
musl += locale/catgets.o
musl += locale/catopen.o
libc += locale/duplocale.o
libc += locale/freelocale.o
musl += locale/iconv.o
musl += locale/intl.o
libc += locale/isalnum_l.o
libc += locale/isalpha_l.o
libc += locale/isblank_l.o
libc += locale/iscntrl_l.o
libc += locale/isdigit_l.o
libc += locale/isgraph_l.o
libc += locale/islower_l.o
libc += locale/isprint_l.o
libc += locale/ispunct_l.o
libc += locale/isspace_l.o
libc += locale/isupper_l.o
libc += locale/iswalnum_l.o
libc += locale/iswalpha_l.o
libc += locale/iswblank_l.o
libc += locale/iswcntrl_l.o
libc += locale/iswctype_l.o
libc += locale/iswdigit_l.o
libc += locale/iswgraph_l.o
libc += locale/iswlower_l.o
libc += locale/iswprint_l.o
libc += locale/iswpunct_l.o
libc += locale/iswspace_l.o
libc += locale/iswupper_l.o
libc += locale/iswxdigit_l.o
libc += locale/isxdigit_l.o
libc += locale/langinfo.o
musl += locale/localeconv.o
#libc += locale/newlocale.o
libc += locale/nl_langinfo_l.o
libc += locale/setlocale.o
musl += locale/strcasecmp_l.o
libc += locale/strcoll.o
libc += locale/strcoll_l.o
musl += locale/strerror_l.o
libc += locale/strfmon.o
libc += locale/strftime_l.o
musl += locale/strncasecmp_l.o
libc += locale/strtod_l.o
libc += locale/strtof_l.o
libc += locale/strtold_l.o
libc += locale/strxfrm.o
libc += locale/strxfrm_l.o
libc += locale/tolower_l.o
libc += locale/toupper_l.o
musl += locale/towctrans_l.o
libc += locale/towlower_l.o
libc += locale/towupper_l.o
libc += locale/uselocale.o
libc += locale/wcscoll.o
libc += locale/wcscoll_l.o
libc += locale/wcsftime_l.o
libc += locale/wcsxfrm.o
libc += locale/wcsxfrm_l.o
musl += locale/wctrans_l.o
libc += locale/wctype_l.o

musl += math/__cos.o
musl += math/__cosdf.o
musl += math/__cosl.o
musl += math/__expo2.o
musl += math/__expo2f.o
musl += math/__fpclassify.o
musl += math/__fpclassifyf.o
musl += math/__fpclassifyl.o
musl += math/__invtrigl.o
musl += math/__polevll.o
musl += math/__rem_pio2.o
musl += math/__rem_pio2_large.o
musl += math/__rem_pio2f.o
musl += math/__rem_pio2l.o
musl += math/__signbit.o
musl += math/__signbitf.o
musl += math/__signbitl.o
musl += math/__sin.o
musl += math/__sindf.o
musl += math/__sinl.o
musl += math/__tan.o
musl += math/__tandf.o
musl += math/__tanl.o
musl += math/acos.o
musl += math/acosf.o
musl += math/acosh.o
musl += math/acoshf.o
musl += math/acoshl.o
musl += math/acosl.o
musl += math/asin.o
musl += math/asinf.o
musl += math/asinh.o
musl += math/asinhf.o
musl += math/asinhl.o
musl += math/asinl.o
musl += math/atan.o
musl += math/atan2.o
musl += math/atan2f.o
musl += math/atan2l.o
musl += math/atanf.o
musl += math/atanh.o
musl += math/atanhf.o
musl += math/atanhl.o
musl += math/atanl.o
musl += math/cbrt.o
musl += math/cbrtf.o
musl += math/cbrtl.o
musl += math/ceil.o
musl += math/ceilf.o
musl += math/ceill.o
musl += math/copysign.o
musl += math/copysignf.o
musl += math/copysignl.o
musl += math/cos.o
musl += math/cosf.o
musl += math/cosh.o
musl += math/coshf.o
musl += math/coshl.o
musl += math/cosl.o
musl += math/erf.o
musl += math/erff.o
musl += math/erfl.o
musl += math/exp.o
musl += math/exp10.o
musl += math/exp10f.o
musl += math/exp10l.o
musl += math/exp2.o
musl += math/exp2f.o
musl += math/exp2l.o
musl/src/math/exp2l.o: CFLAGS += -Wno-error=unused-variable
musl += math/expf.o
musl += math/expl.o
musl += math/expm1.o
musl += math/expm1f.o
musl += math/expm1l.o
musl += math/fabs.o
musl += math/fabsf.o
musl += math/fabsl.o
musl += math/fdim.o
musl += math/fdimf.o
musl += math/fdiml.o
musl += math/floor.o
musl += math/floorf.o
musl += math/floorl.o
#musl += math/fma.o
#musl += math/fmaf.o
#musl += math/fmal.o
musl += math/fmax.o
musl += math/fmaxf.o
musl += math/fmaxl.o
musl += math/fmin.o
musl += math/fminf.o
musl += math/fminl.o
musl += math/fmod.o
musl += math/fmodf.o
musl += math/fmodl.o
libc += math/finite.o
libc += math/finitef.o
libc += math/finitel.o
musl += math/frexp.o
musl += math/frexpf.o
musl += math/frexpl.o
musl += math/hypot.o
musl += math/hypotf.o
musl += math/hypotl.o
musl += math/ilogb.o
musl += math/ilogbf.o
musl += math/ilogbl.o
musl += math/j0.o
musl += math/j0f.o
musl += math/j1.o
musl += math/j1f.o
musl += math/jn.o
musl += math/jnf.o
musl += math/ldexp.o
musl += math/ldexpf.o
musl += math/ldexpl.o
musl += math/lgamma.o
musl += math/lgamma_r.o
musl/src/math/lgamma_r.o: CFLAGS += -Wno-error=maybe-uninitialized
musl += math/lgammaf.o
musl += math/lgammaf_r.o
musl/src/math/lgammaf_r.o: CFLAGS += -Wno-error=maybe-uninitialized
musl += math/lgammal.o
musl/src/math/lgammal.o: CFLAGS += -Wno-error=maybe-uninitialized
#musl += math/llrint.o
#musl += math/llrintf.o
#musl += math/llrintl.o
musl += math/llround.o
musl += math/llroundf.o
musl += math/llroundl.o
musl += math/log.o
musl += math/log10.o
musl += math/log10f.o
musl += math/log10l.o
musl += math/log1p.o
musl += math/log1pf.o
musl += math/log1pl.o
musl += math/log2.o
musl += math/log2f.o
musl += math/log2l.o
musl += math/logb.o
musl += math/logbf.o
musl += math/logbl.o
musl += math/logf.o
musl += math/logl.o
musl += math/lrint.o
#musl += math/lrintf.o
#musl += math/lrintl.o
musl += math/lround.o
musl += math/lroundf.o
musl += math/lroundl.o
musl += math/modf.o
musl += math/modff.o
musl += math/modfl.o
musl += math/nan.o
musl += math/nanf.o
musl += math/nanl.o
#musl += math/nearbyint.o
#musl += math/nearbyintf.o
#musl += math/nearbyintl.o
musl += math/nextafter.o
musl += math/nextafterf.o
musl += math/nextafterl.o
musl += math/nexttoward.o
musl += math/nexttowardf.o
musl += math/nexttowardl.o
musl += math/pow.o
musl += math/powf.o
musl += math/powl.o
musl += math/remainder.o
musl += math/remainderf.o
musl += math/remainderl.o
musl += math/remquo.o
musl += math/remquof.o
musl += math/remquol.o
musl += math/rint.o
musl += math/rintf.o
musl += math/rintl.o
musl += math/round.o
musl += math/roundf.o
musl += math/roundl.o
musl += math/scalb.o
musl += math/scalbf.o
musl += math/scalbln.o
musl += math/scalblnf.o
musl += math/scalblnl.o
musl += math/scalbn.o
musl += math/scalbnf.o
musl += math/scalbnl.o
musl += math/signgam.o
musl += math/significand.o
musl += math/significandf.o
musl += math/sin.o
musl += math/sincos.o
musl += math/sincosf.o
msul += math/sincosl.o
musl += math/sinf.o
musl += math/sinh.o
musl += math/sinhf.o
musl += math/sinhl.o
musl += math/sinl.o
musl += math/sqrt.o
musl += math/sqrtf.o
musl += math/sqrtl.o
musl += math/tan.o
musl += math/tanf.o
musl += math/tanh.o
musl += math/tanhf.o
musl += math/tanhl.o
musl += math/tanl.o
musl += math/tgamma.o
musl += math/tgammaf.o
musl += math/tgammal.o
musl += math/trunc.o
musl += math/truncf.o
musl += math/truncl.o

musl += misc/a64l.o
libc += misc/basename.o
musl += misc/dirname.o
libc += misc/ffs.o
musl += misc/get_current_dir_name.o
musl += misc/gethostid.o
musl += misc/getopt.o
musl += misc/getopt_long.o
musl += misc/getsubopt.o
libc += misc/realpath.o
libc += misc/backtrace.o
libc += misc/uname.o
libc += misc/lockf.o
libc += misc/mntent.o
libc += misc/__longjmp_chk.o

musl += multibyte/btowc.o
musl += multibyte/internal.o
musl += multibyte/mblen.o
musl += multibyte/mbrlen.o
musl += multibyte/mbrtowc.o
musl += multibyte/mbsinit.o
musl += multibyte/mbsnrtowcs.o
libc += multibyte/mbsrtowcs.o
musl += multibyte/mbstowcs.o
musl += multibyte/mbtowc.o
musl += multibyte/wcrtomb.o
musl += multibyte/wcsnrtombs.o
musl += multibyte/wcsrtombs.o
musl += multibyte/wcstombs.o
musl += multibyte/wctob.o
musl += multibyte/wctomb.o

libc/multibyte/mbsrtowcs.o: CFLAGS += -I $(src)/musl/src/multibyte

libc += network/htonl.o
libc += network/htons.o
libc += network/ntohl.o
libc += network/ntohs.o
libc += network/gethostbyname_r.o
musl += network/gethostbyname2_r.o
musl += network/gethostbyaddr_r.o
musl += network/gethostbyaddr.o
libc += network/getaddrinfo.o
musl += network/freeaddrinfo.o
musl += network/in6addr_any.o
libc += network/getnameinfo.o
libc += network/__dns.o
libc += network/__ipparse.o
libc += network/inet_addr.o
libc += network/inet_aton.o
musl += network/inet_pton.o
libc += network/inet_ntop.o
musl += network/proto.o
libc += network/if_indextoname.o
libc += network/if_nametoindex.o
libc += network/gai_strerror.o
libc += network/h_errno.o
musl += network/getservbyname_r.o
musl += network/getservbyname.o
musl += network/getservbyport_r.o
musl += network/getifaddrs.o
musl += network/if_nameindex.o
musl += network/if_freenameindex.o

libc += prng/rand.o
libc += prng/random.o

libc += process/execve.o
libc += process/execle.o
musl += process/execv.o
musl += process/execl.o
libc += process/waitpid.o

libc += arch/$(arch)/setjmp/setjmp.o
libc += arch/$(arch)/setjmp/longjmp.o
libc += arch/$(arch)/setjmp/sigrtmax.o
libc += arch/$(arch)/setjmp/sigrtmin.o
libc += arch/$(arch)/setjmp/siglongjmp.o
libc += arch/$(arch)/setjmp/sigsetjmp.o
libc += arch/$(arch)/setjmp/block.o
ifeq ($(arch),x64)
libc += arch/$(arch)/ucontext/getcontext.o
libc += arch/$(arch)/ucontext/setcontext.o
libc += arch/$(arch)/ucontext/start_context.o
libc += arch/$(arch)/ucontext/ucontext.o
endif

musl += stdio/__fclose_ca.o
libc += stdio/__fdopen.o
musl += stdio/__fmodeflags.o
libc += stdio/__fopen_rb_ca.o
libc += stdio/__fprintf_chk.o
libc += stdio/__lockfile.o
musl += stdio/__overflow.o
libc += stdio/__stdio_close.o
musl += stdio/__stdio_exit.o
libc += stdio/__stdio_read.o
libc += stdio/__stdio_seek.o
libc += stdio/__stdio_write.o
libc += stdio/__stdout_write.o
musl += stdio/__string_read.o
musl += stdio/__toread.o
musl += stdio/__towrite.o
musl += stdio/__uflow.o
libc += stdio/__vfprintf_chk.o
musl += stdio/asprintf.o
musl += stdio/clearerr.o
musl += stdio/dprintf.o
musl += stdio/ext.o
musl += stdio/ext2.o
musl += stdio/fclose.o
musl += stdio/feof.o
musl += stdio/ferror.o
musl += stdio/fflush.o
libc += stdio/fgetc.o
musl += stdio/fgetln.o
musl += stdio/fgetpos.o
musl += stdio/fgets.o
musl += stdio/fgetwc.o
musl += stdio/fgetws.o
musl += stdio/fileno.o
libc += stdio/flockfile.o
libc += stdio/fmemopen.o
libc += stdio/fopen.o
musl += stdio/fprintf.o
libc += stdio/fputc.o
musl += stdio/fputs.o
musl += stdio/fputwc.o
musl += stdio/fputws.o
musl += stdio/fread.o
libc += stdio/__fread_chk.o
libc += stdio/freopen.o
musl += stdio/fscanf.o
musl += stdio/fseek.o
musl += stdio/fsetpos.o
musl += stdio/ftell.o
libc += stdio/ftrylockfile.o
libc += stdio/funlockfile.o
musl += stdio/fwide.o
musl += stdio/fwprintf.o
musl += stdio/fwrite.o
musl += stdio/fwscanf.o
libc += stdio/getc.o
musl += stdio/getc_unlocked.o
musl += stdio/getchar.o
musl += stdio/getchar_unlocked.o
musl += stdio/getdelim.o
musl += stdio/getline.o
musl += stdio/gets.o
musl += stdio/getw.o
musl += stdio/getwc.o
musl += stdio/getwchar.o
libc += stdio/open_memstream.o
libc += stdio/open_wmemstream.o
musl += stdio/perror.o
musl += stdio/printf.o
libc += stdio/putc.o
musl += stdio/putc_unlocked.o
musl += stdio/putchar.o
musl += stdio/putchar_unlocked.o
musl += stdio/puts.o
musl += stdio/putw.o
musl += stdio/putwc.o
musl += stdio/putwchar.o
libc += stdio/remove.o
musl += stdio/rewind.o
musl += stdio/scanf.o
musl += stdio/setbuf.o
musl += stdio/setbuffer.o
musl += stdio/setlinebuf.o
musl += stdio/setvbuf.o
musl += stdio/snprintf.o
musl += stdio/sprintf.o
musl += stdio/sscanf.o
libc += stdio/stderr.o
libc += stdio/stdin.o
libc += stdio/stdout.o
musl += stdio/swprintf.o
musl += stdio/swscanf.o
musl += stdio/tempnam.o
libc += stdio/tmpfile.o
libc += stdio/tmpnam.o
musl += stdio/ungetc.o
musl += stdio/ungetwc.o
musl += stdio/vasprintf.o
libc += stdio/vdprintf.o
musl += stdio/vfprintf.o
libc += stdio/vfscanf.o
musl += stdio/vfwprintf.o
libc += stdio/vfwscanf.o
musl += stdio/vprintf.o
musl += stdio/vscanf.o
libc += stdio/vsnprintf.o
musl += stdio/vsprintf.o
libc += stdio/vsscanf.o
libc += stdio/vswprintf.o
libc += stdio/vswscanf.o
musl += stdio/vwprintf.o
musl += stdio/vwscanf.o
musl += stdio/wprintf.o
musl += stdio/wscanf.o

musl += stdlib/abs.o
musl += stdlib/atof.o
musl += stdlib/atoi.o
musl += stdlib/atol.o
musl += stdlib/atoll.o
musl += stdlib/bsearch.o
musl += stdlib/div.o
musl += stdlib/ecvt.o
libc += stdlib/fcvt.o
musl += stdlib/gcvt.o
musl += stdlib/imaxabs.o
musl += stdlib/imaxdiv.o
musl += stdlib/labs.o
musl += stdlib/ldiv.o
musl += stdlib/llabs.o
musl += stdlib/lldiv.o
musl += stdlib/qsort.o
libc += stdlib/strtol.o
libc += stdlib/strtod.o
libc += stdlib/wcstol.o

libc += string/__memcpy_chk.o
musl += string/bcmp.o
musl += string/bcopy.o
musl += string/bzero.o
musl += string/index.o
libc += string/memccpy.o
libc += string/memchr.o
musl += string/memcmp.o
libc += string/memcpy.o
musl += string/memmem.o
libc += string/memmove.o
musl += string/mempcpy.o
musl += string/memrchr.o
libc += string/__memmove_chk.o
libc += string/memset.o
libc += string/__memset_chk.o
libc += string/rawmemchr.o
musl += string/rindex.o
libc += string/stpcpy.o
libc += string/__stpcpy_chk.o
libc += string/stpncpy.o
musl += string/strcasecmp.o
musl += string/strcasestr.o
libc += string/strcat.o
libc += string/strchr.o
libc += string/strchrnul.o
libc += string/strcmp.o
libc += string/strcpy.o
libc += string/__strcpy_chk.o
libc += string/strcspn.o
libc += string/strdup.o
libc += string/strerror_r.o
libc += string/strlcat.o
libc += string/strlcpy.o
libc += string/strlen.o
musl += string/strncasecmp.o
libc += string/strncat.o
libc += string/__strncat_chk.o
libc += string/strncmp.o
libc += string/strncpy.o
libc += string/__strndup.o
musl += string/strndup.o
musl += string/strnlen.o
libc += string/strpbrk.o
musl += string/strrchr.o
libc += string/strsep.o
libc += string/strsignal.o
libc += string/strspn.o
musl += string/strstr.o
libc += string/strtok.o
libc += string/strtok_r.o
musl += string/strverscmp.o
libc += string/swab.o
libc += string/wcpcpy.o
libc += string/wcpncpy.o
musl += string/wcscasecmp.o
musl += string/wcscasecmp_l.o
libc += string/wcscat.o
musl += string/wcschr.o
musl += string/wcscmp.o
libc += string/wcscpy.o
musl += string/wcscspn.o
musl += string/wcsdup.o
musl += string/wcslen.o
musl += string/wcsncasecmp.o
musl += string/wcsncasecmp_l.o
libc += string/wcsncat.o
musl += string/wcsncmp.o
libc += string/wcsncpy.o
musl += string/wcsnlen.o
musl += string/wcspbrk.o
musl += string/wcsrchr.o
musl += string/wcsspn.o
libc += string/wcsstr.o
libc += string/wcstok.o
musl += string/wcswcs.o
musl += string/wmemchr.o
musl += string/wmemcmp.o
libc += string/wmemcpy.o
musl += string/wmemmove.o
musl += string/wmemset.o

musl += temp/__randname.o
musl += temp/mkdtemp.o
musl += temp/mkstemp.o
musl += temp/mktemp.o
musl += temp/mkostemps.o

libc += time/__asctime.o
libc += time/__time_to_tm.o
libc += time/__tm_to_time.o
musl += time/asctime.o
musl += time/asctime_r.o
musl += time/ctime.o
musl += time/ctime_r.o
musl += time/difftime.o
libc += time/getdate.o
libc += time/gmtime.o
libc += time/gmtime_r.o
libc += time/localtime.o
libc += time/localtime_r.o
libc += time/mktime.o
libc += time/strftime.o
libc += time/strptime.o
libc += time/time.o
libc += time/timegm.o
libc += time/tzset.o
libc += time/wcsftime.o
libc += time/ftime.o # verbatim copy of the file as in 4b15d9f46a2b@musl
libc/time/ftime.o: CFLAGS += -I $(src)/libc/include

musl += unistd/sleep.o
musl += unistd/gethostname.o
libc += unistd/sethostname.o
libc += unistd/sync.o
libc += unistd/getpgid.o
libc += unistd/setpgid.o
libc += unistd/getpgrp.o
libc += unistd/getppid.o
libc += unistd/getsid.o
libc += unistd/setsid.o

libc += regex/fnmatch.o
libc += regex/glob.o

libc += pthread.o
libc += libc.o
libc += dlfcn.o
libc += time.o
libc += signal.o
libc += mman.o
libc += sem.o
libc += pipe_buffer.o
libc += pipe.o
libc += af_local.o
libc += user.o
libc += resource.o
libc += mount.o
libc += eventfd.o
libc += timerfd.o
libc += shm.o
libc += inotify.o

ifneq ($(musl_arch), notsup)
musl += fenv/fegetexceptflag.o
musl += fenv/feholdexcept.o
musl += fenv/fesetexceptflag.o
musl += fenv/$(musl_arch)/fenv.o
endif
