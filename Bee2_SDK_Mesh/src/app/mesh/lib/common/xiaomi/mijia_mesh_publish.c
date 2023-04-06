/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mesh_publish.c
* @brief    Source file for mijia mesh publish
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-09-09
* @version  v1.0
* *************************************************************************************
*/
#include <stdlib.h>
#include "mijia_mesh_config.h"
#include "mijia_mesh_publish.h"
#include "mijia_mesh_app.h"

#define MI_STARTUP_FIRST_PUB_PERIOD          10000

typedef struct _mi_pub_list
{
    struct _mi_pub_list *pprev;
    struct _mi_pub_list *pnext;
} mi_pub_list_t;

typedef struct
{
    mesh_model_info_t *pmodel_info;
    uint32_t cur_pub_period;
    plt_timer_t pub_timer;
    mi_pub_list_t node;
} mi_pub_t;

/* periodic publish parameters */
static plt_timer_t mi_pub_timer;
/* default publish period 3 miniutes */
#define MI_PUB_PERIOD_RAND_TIME_MIN           1000
#define MI_PUB_PERIOD_RAND_TIME_MAX           10000
static uint32_t mi_max_pub_period = 180000;
static uint32_t mi_cur_pub_period;
static mi_pub_list_t *pmi_cur_pub;
static mi_pub_list_t mi_pub_list_head;

/* publish parameters after state change */
#define MI_PUB_SINGLE_INIT_TIME_MIN           10000
#define MI_PUB_SINGLE_INIT_TIME_MAX           20000
static mi_pub_list_t mi_pub_single_list_head;

static bool mi_pub_init = FALSE;

static uint32_t mi_pub_random_get(uint32_t low, uint32_t high)
{
    int delay = rand();
    uint32_t real_delay = delay;
    /* random delay low-high */
    real_delay %= high;
    if (real_delay < low)
    {
        real_delay += low;
    }

    return real_delay;
}

void mi_model_publish_set(mesh_model_p pmodel, uint16_t address, uint16_t app_key_idx_g)
{
    if (MESH_IS_VIRTUAL_ADDR(address))
    {
        vir_addr_delete(address & 0x3fff);
    }

    mesh_model_pub_params_t pub_params;
    pub_params.pub_addr = address;
    pub_params.pub_key_info.app_key_index = app_key_idx_g;
    pub_params.pub_key_info.frnd_flag = 0;
    pub_params.pub_key_info.rfu = 0;
    pub_params.pub_ttl = MI_DEFAULT_TTL;
    pub_params.pub_period.steps = MI_PUB_NUM_STEPS;
    pub_params.pub_period.resol = MI_PUB_STEP_RESOLUTION;
    pub_params.pub_retrans_info.count = MI_PUB_RETRANS_COUNT;
    pub_params.pub_retrans_info.steps = MI_PUB_RETRANS_INTERVAL_STEPS;
    mesh_model_pub_params_set(pmodel, pub_params);
}

static void mi_pub_add_to_periodic(mi_pub_t *ppub)
{
    /* append to list */
    mi_pub_list_head.pprev->pnext = &ppub->node;
    ppub->node.pprev = mi_pub_list_head.pprev;
    mi_pub_list_head.pprev = &ppub->node;
    ppub->node.pnext = &mi_pub_list_head;

    if (NULL == pmi_cur_pub)
    {
        pmi_cur_pub = &ppub->node;
    }
}

static void mi_pub_add_to_single(mi_pub_t *ppub)
{
    mi_pub_single_list_head.pprev->pnext = &ppub->node;
    ppub->node.pprev = mi_pub_single_list_head.pprev;
    mi_pub_single_list_head.pprev = &ppub->node;
    ppub->node.pnext = &mi_pub_single_list_head;
}

static void mi_pub_remove(mi_pub_t *ppub)
{
    /* remove from list */
    ppub->node.pprev->pnext = ppub->node.pnext;
    ppub->node.pnext->pprev = ppub->node.pprev;
    ppub->node.pnext = NULL;
    ppub->node.pprev = NULL;
}

mi_pub_t *mi_pub_get(mi_pub_list_t *phead, mesh_model_info_t *pmodel_info)
{
    mi_pub_list_t *pnode = phead->pnext;
    mi_pub_t *ppub;
    for (; pnode != phead; pnode = pnode->pnext)
    {
        ppub = CONTAINER_OF(pnode, mi_pub_t, node);
        if (ppub->pmodel_info == pmodel_info)
        {
            return ppub;
        }
    }

    return NULL;
}

mi_pub_t *mi_pub_single_get(plt_timer_t ptimer)
{
    mi_pub_list_t *pnode = mi_pub_single_list_head.pnext;
    mi_pub_t *ppub;
    for (; pnode != &mi_pub_single_list_head; pnode = pnode->pnext)
    {
        ppub = CONTAINER_OF(pnode, mi_pub_t, node);
        if (ppub->pub_timer == ptimer)
        {
            return ppub;
        }
    }

    return NULL;
}

static void mi_pub_try_to_restart(void)
{
    /* check empty */
    if (mi_pub_single_list_head.pnext == &mi_pub_single_list_head)
    {
        /* start periodic publish */
        if (!plt_timer_is_active(mi_pub_timer))
        {
            plt_timer_start(mi_pub_timer, 0);
        }
    }
}

void mi_process_publish_single_timeout(plt_timer_t ptimer)
{
    mi_pub_t *ppub = mi_pub_single_get(ptimer);
    if (NULL != ppub)
    {
        if (NULL != ppub->pmodel_info->model_pub_cb)
        {
            ppub->pmodel_info->model_pub_cb(ppub->pmodel_info, FALSE);
        }
        else
        {
            /* invalid model publish */
            printw("missing model defalut publish callback: id 0x%08x!", ppub->pmodel_info->model_id);
        }
        if (ppub->cur_pub_period < mi_max_pub_period)
        {
            ppub->cur_pub_period = ppub->cur_pub_period * 2 + mi_pub_random_get(MI_PUB_PERIOD_RAND_TIME_MIN,
                                                                                MI_PUB_PERIOD_RAND_TIME_MAX);
            if (ppub->cur_pub_period > mi_max_pub_period)
            {
                /* remove from current list */
                mi_pub_remove(ppub);
                if (NULL != ppub->pub_timer)
                {
                    plt_timer_delete(ppub->pub_timer, 0);
                    ppub->pub_timer = NULL;
                }
                mi_pub_add_to_periodic(ppub);
                mi_pub_try_to_restart();
            }
            else
            {
                plt_timer_change_period(ppub->pub_timer, ppub->cur_pub_period, 0);
            }
        }
        else
        {
            /* remove from current list */
            mi_pub_remove(ppub);
            if (NULL != ppub->pub_timer)
            {
                plt_timer_delete(ppub->pub_timer, 0);
                ppub->pub_timer = NULL;
            }
            mi_pub_add_to_periodic(ppub);
            mi_pub_try_to_restart();
        }
    }
}

static void mi_pub_single_timeout_cb(void *ptimer)
{
    mi_inner_msg_t msg;
    msg.type = MI_PUB_SINGLE_TIMEOUT;
    msg.pbuf = ptimer;
    mi_inner_msg_send(&msg);
}

bool mi_publish_single_start(mesh_model_info_t *pmodel_info)
{
    /* find from current single publish list */
    mi_pub_t *ppub = mi_pub_get(&mi_pub_single_list_head, pmodel_info);
    if (NULL == ppub)
    {
        /* find from current periodic publish list */
        ppub = mi_pub_get(&mi_pub_list_head, pmodel_info);
        if (NULL == ppub)
        {
            printe("mi_publish_single_start: invalid model 0x%08x", pmodel_info->model_id);
            return FALSE;
        }

        /* remove current periodic publish */
        if (pmi_cur_pub == &ppub->node)
        {
            if (mi_pub_list_head.pnext->pnext == &mi_pub_list_head)
            {
                /* only one element */
                pmi_cur_pub = NULL;
            }
            else
            {
                pmi_cur_pub = pmi_cur_pub->pnext;
                if (pmi_cur_pub == &mi_pub_list_head)
                {
                    pmi_cur_pub = pmi_cur_pub->pnext;
                }
            }
        }

        /* stop periodic publish */
        if (plt_timer_is_active(mi_pub_timer))
        {
            plt_timer_stop(mi_pub_timer, 0);
        }

        mi_pub_remove(ppub);
        mi_pub_add_to_single(ppub);

        ppub->cur_pub_period = mi_pub_random_get(MI_PUB_SINGLE_INIT_TIME_MIN, MI_PUB_SINGLE_INIT_TIME_MAX);
        if (NULL == ppub->pub_timer)
        {

            ppub->pub_timer = plt_timer_create("pub_s", ppub->cur_pub_period, FALSE, 0,
                                               mi_pub_single_timeout_cb);
            if (NULL != ppub->pub_timer)
            {
                plt_timer_start(ppub->pub_timer, 0);
            }
            else
            {
                mi_pub_remove(ppub);
                mi_pub_add_to_periodic(ppub);
                mi_pub_try_to_restart();
            }
        }
        else
        {
            plt_timer_change_period(ppub->pub_timer, ppub->cur_pub_period, 0);
        }
    }
    else
    {
        /* reset single publish parameters */
        ppub->cur_pub_period = mi_pub_random_get(MI_PUB_SINGLE_INIT_TIME_MIN, MI_PUB_SINGLE_INIT_TIME_MAX);
        plt_timer_change_period(ppub->pub_timer, ppub->cur_pub_period, 0);
    }

    return TRUE;
}

static void mi_publish_single_stop_all(void)
{
    mi_pub_list_t *pnode = mi_pub_single_list_head.pnext;
    mi_pub_t *ppub = NULL;
    while (pnode != &mi_pub_single_list_head)
    {
        ppub = CONTAINER_OF(pnode, mi_pub_t, node);
        if (NULL != ppub->pub_timer)
        {
            plt_timer_delete(ppub->pub_timer, 0);
            ppub->pub_timer = NULL;
        }
        pnode = pnode->pnext;
        mi_pub_add_to_periodic(ppub);
    }

    mi_pub_single_list_head.pprev = &mi_pub_single_list_head;
    mi_pub_single_list_head.pnext = &mi_pub_single_list_head;
}

void mi_process_publish_timeout(void)
{
    mi_pub_t *ppub = NULL;
    if (0 == mi_cur_pub_period)
    {
        /* publish all status */
        mi_pub_list_t *pnode = mi_pub_list_head.pnext;
        for (; pnode != &mi_pub_list_head; pnode = pnode->pnext)
        {
            ppub = CONTAINER_OF(pnode, mi_pub_t, node);
            if (NULL != ppub->pmodel_info->model_pub_cb)
            {
                ppub->pmodel_info->model_pub_cb(ppub->pmodel_info, FALSE);
            }
        }
    }
    else
    {
        /* publish specified status */
        if (NULL != pmi_cur_pub)
        {
            mi_pub_t *ppub = CONTAINER_OF(pmi_cur_pub, mi_pub_t, node);
            if (NULL != ppub->pmodel_info->model_pub_cb)
            {
                ppub->pmodel_info->model_pub_cb(ppub->pmodel_info, FALSE);
            }
            else
            {
                /* invalid model publish */
                printw("missing model defalut publish callback: id 0x%08x!", ppub->pmodel_info->model_id);
            }

            /* adjust publish index */
            pmi_cur_pub = pmi_cur_pub->pnext;
            if (pmi_cur_pub == &mi_pub_list_head)
            {
                pmi_cur_pub = pmi_cur_pub->pnext;
            }
        }
    }

    if (mi_cur_pub_period < mi_max_pub_period)
    {
        if (0 == mi_cur_pub_period)
        {
            mi_cur_pub_period = mi_pub_random_get(MI_PUB_PERIOD_RAND_TIME_MIN, MI_PUB_PERIOD_RAND_TIME_MAX);
        }
        else
        {
            mi_cur_pub_period = mi_cur_pub_period * 2 + mi_pub_random_get(MI_PUB_PERIOD_RAND_TIME_MIN,
                                                                          MI_PUB_PERIOD_RAND_TIME_MAX);
            if (mi_cur_pub_period > mi_max_pub_period)
            {
                mi_cur_pub_period = mi_max_pub_period;
            }
        }
    }
    else
    {
        mi_cur_pub_period = mi_max_pub_period;
    }
    plt_timer_change_period(mi_pub_timer, mi_cur_pub_period, 0);
}

static void mi_pub_timeout_cb(void *ptimer)
{
    mi_inner_msg_t msg;
    msg.type = MI_PUB_TIMEOUT;
    mi_inner_msg_send(&msg);
}

bool mi_publish_start(void)
{
    if (NULL != mi_pub_timer)
    {
        if (plt_timer_is_active(mi_pub_timer))
        {
            plt_timer_stop(mi_pub_timer, 0);
        }
    }

    mesh_element_t *pelement = (mesh_element_t *)mesh_node.element_queue.pfirst;
    while (NULL != pelement)
    {
        mesh_model_t *pmodel = (mesh_model_t *)pelement->model_queue.pfirst;
        while (NULL != pmodel)
        {
            for (uint16_t index = 0; index < mesh_node.app_key_num; index++)
            {
                if (plt_bit_pool_get(pmodel->app_key_binding, index) &&
                    mesh_node.app_key_list[index].key_state != MESH_KEY_STATE_INVALID)
                {
                    mi_model_publish_set(pmodel, MI_PUB_ADDRESS, app_key_index_to_global(index));
                    if (pmodel->pmodel_info->model_pub_cb)
                    {
                        /* add new */
                        mi_pub_t *ppub = plt_malloc(sizeof(mi_pub_t), RAM_TYPE_DATA_ON);
                        if (NULL != ppub)
                        {
                            ppub->pmodel_info = pmodel->pmodel_info;
                            ppub->cur_pub_period = 0;
                            ppub->pub_timer = NULL;
                            mi_pub_add_to_periodic(ppub);
                        }
                    }
                    break;
                }
            }
            pmodel = pmodel->pnext;
        }
        pelement = pelement->pnext;
    }

    if (mi_pub_list_head.pnext != &mi_pub_list_head)
    {
        pmi_cur_pub = mi_pub_list_head.pnext;
    }
    else
    {
        pmi_cur_pub = NULL;
    }

    if (NULL == mi_pub_timer)
    {
        /* create pub timer */
        mi_pub_timer = plt_timer_create("pub", 10, FALSE, 0, mi_pub_timeout_cb);
        if (NULL == mi_pub_timer)
        {
            return FALSE;
        }
    }
    plt_timer_start(mi_pub_timer, 0);

    return TRUE;
}

void mi_publish_stop(void)
{
    if (!mi_pub_init)
    {
        return ;
    }

    mi_publish_single_stop_all();

    if (NULL != mi_pub_timer)
    {
        plt_timer_delete(mi_pub_timer, 0);
        mi_pub_timer = NULL;
    }

    /* remove all node from list */
    mi_pub_list_t *pnode = mi_pub_list_head.pnext;
    mi_pub_t *ppub = NULL;
    while (pnode != &mi_pub_list_head)
    {
        ppub = CONTAINER_OF(pnode, mi_pub_t, node);
        pnode = pnode->pnext;
        plt_free(ppub, RAM_TYPE_DATA_ON);
    }
    mi_pub_list_head.pprev = &mi_pub_list_head;
    mi_pub_list_head.pnext = &mi_pub_list_head;

    pmi_cur_pub = NULL;
    mi_cur_pub_period = 0;
}

void mi_publish_interval_set(uint32_t interval)
{
    mi_max_pub_period = interval * 60000;
}

bool mi_publish_init(void)
{
    if ((NULL == mi_pub_single_list_head.pprev) &&
        (NULL == mi_pub_single_list_head.pnext))
    {
        mi_pub_single_list_head.pprev = &mi_pub_single_list_head;
        mi_pub_single_list_head.pnext = &mi_pub_single_list_head;
    }


    if ((NULL == mi_pub_list_head.pprev) &&
        (NULL == mi_pub_list_head.pnext))
    {
        mi_pub_list_head.pprev = &mi_pub_list_head;
        mi_pub_list_head.pnext = &mi_pub_list_head;
    }

    mi_pub_init = TRUE;

    return TRUE;
}
