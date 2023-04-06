#ifndef __SE_APDU_H__
#define __SE_APDU_H__
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned char *cmd;
    unsigned int  clen;
    unsigned char *out;
    unsigned int  olen; /* the size of out buffer */
    unsigned int  dlen; /* the size of out data */
} apdu_command_t;

/* 
    return 0 on success, otherwise you SHOULD check output SW code.
 */
int se_apdu_command(apdu_command_t *command);

#ifdef __cplusplus
}
#endif
#endif