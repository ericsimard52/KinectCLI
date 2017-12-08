#ifndef __dbg_h__
#define __dbg_h__

#include <stdio.h>
#include <errno.h>
#include <string.h>

#define __output__ stdout

char *USER_ERR_MSG;

#define debug(M, ...) fprintf(__output__, ":> " M "\n", ##__VA_ARGS__)


#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) fprintf(__output__, "[ERROR] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) fprintf(__output__, "[WARN] (%s:%d errno: %s)" M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) fprintf(__output__, "[INFO] (%s:%d errno: %s)" M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define check(A, M, ...) if (!(A)){ log_err(M, ##__VA_ARGS__); errno=0; USER_ERR_MSG = malloc(sizeof(char) * 1024); snprintf(USER_ERR_MSG, 1024, "%s", M); goto error; }

#define sentinel(M, ...) { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define check_mem(A) check((A), "Out of memory.")

#define check_debug(A, M, ...) if(!(A)) { debug(M, ##__VA_ARGS__); errno=0; goto error; }

#endif


