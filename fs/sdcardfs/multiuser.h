/*
 * fs/sdcardfs/multiuser.h
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun, 
 *               Sunghwan Yun, Sungjong Seo
 *                      
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by 
<<<<<<< HEAD
=======
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

<<<<<<< HEAD
<<<<<<< HEAD
#define MULTIUSER_APP_PER_USER_RANGE 100000
=======
#define AID_USER_OFFSET     100000 /* offset for uid ranges for each user */
#define AID_APP_START        10000 /* first app user */
#define AID_APP_END          19999 /* last app user */
#define AID_CACHE_GID_START  20000 /* start of gids for apps to mark cached data */
#define AID_EXT_GID_START    30000 /* start of gids for apps to mark external data */
#define AID_SHARED_GID_START 50000 /* start of gids for apps in each user to share */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
#define MULTIUSER_APP_PER_USER_RANGE 100000
>>>>>>> 2617302... source

typedef uid_t userid_t;
typedef uid_t appid_t;

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
static inline userid_t multiuser_get_user_id(uid_t uid) {
    return uid / MULTIUSER_APP_PER_USER_RANGE;
}       
        
static inline appid_t multiuser_get_app_id(uid_t uid) {
    return uid % MULTIUSER_APP_PER_USER_RANGE;
}       
    
static inline uid_t multiuser_get_uid(userid_t userId, appid_t appId) {
    return userId * MULTIUSER_APP_PER_USER_RANGE + (appId % MULTIUSER_APP_PER_USER_RANGE);
}

<<<<<<< HEAD
=======
static inline uid_t multiuser_get_uid(userid_t user_id, appid_t app_id)
{
	return (user_id * AID_USER_OFFSET) + (app_id % AID_USER_OFFSET);
}

static inline gid_t multiuser_get_cache_gid(uid_t uid)
{
	return uid - AID_APP_START + AID_CACHE_GID_START;
}

static inline gid_t multiuser_get_ext_gid(uid_t uid)
{
	return uid - AID_APP_START + AID_EXT_GID_START;
}
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
