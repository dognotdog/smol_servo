/**
 * lwjson options for smol-servo. lwjson_opt.h includes this file.
 *
 * Only the stream parser is in the build. The configuration file is a flat JSON object, so the stack needs 2 entries (object, key). The other values are the lwjson defaults.
 */
#ifndef LWJSON_OPTS_HDR_H
#define LWJSON_OPTS_HDR_H

#define LWJSON_CFG_STREAM_STACK_SIZE 4
#define LWJSON_CFG_STREAM_KEY_MAX_LEN 32
#define LWJSON_CFG_STREAM_STRING_MAX_LEN 64
#define LWJSON_CFG_STREAM_PRIMITIVE_MAX_LEN 32

#endif /* LWJSON_OPTS_HDR_H */
