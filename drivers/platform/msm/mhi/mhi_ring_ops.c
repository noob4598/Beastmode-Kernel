<<<<<<< HEAD
<<<<<<< HEAD
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
>>>>>>> 2617302... source
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "mhi_sys.h"
#include "mhi.h"

<<<<<<< HEAD
<<<<<<< HEAD
static enum MHI_STATUS add_element(struct mhi_ring *ring, void **rp,
			void **wp, void **assigned_addr)
=======
inline MHI_STATUS ctxt_add_element(mhi_ring *ring, void **assigned_addr)
{
	return add_element(ring, &ring->rp, &ring->wp, assigned_addr);
}
inline MHI_STATUS ctxt_del_element(mhi_ring *ring, void **assigned_addr)
{
	return delete_element(ring, &ring->rp, &ring->wp, assigned_addr);
}

MHI_STATUS add_element(mhi_ring *ring, void * volatile *rp,
			void * volatile *wp, void **assigned_addr)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
static enum MHI_STATUS add_element(struct mhi_ring *ring, void **rp,
			void **wp, void **assigned_addr)
>>>>>>> 2617302... source
{
	uintptr_t d_wp = 0, d_rp = 0, ring_size = 0;

	if (0 == ring->el_size || NULL == ring
		|| NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}

	if (MHI_STATUS_SUCCESS != get_element_index(ring, *rp, &d_rp)) {
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != get_element_index(ring, *wp, &d_wp)) {
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
		return MHI_STATUS_ERROR;
	}
	ring_size = ring->len / ring->el_size;

	if ((d_wp + 1) % ring_size == d_rp) {
		if (ring->overwrite_en) {
			ctxt_del_element(ring, NULL);
		} else {
			mhi_log(MHI_MSG_INFO, "Ring 0x%lX is full\n",
					(uintptr_t)ring->base);
			return MHI_STATUS_RING_FULL;
		}
	}
	if (NULL != assigned_addr)
		*assigned_addr = (char *)ring->wp;
	*wp = (void *)(((d_wp + 1) % ring_size) * ring->el_size +
						(uintptr_t)ring->base);
	return MHI_STATUS_SUCCESS;
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source

inline enum MHI_STATUS ctxt_add_element(struct mhi_ring *ring,
						void **assigned_addr)
{
	return add_element(ring, &ring->rp, &ring->wp, assigned_addr);
}
inline enum MHI_STATUS ctxt_del_element(struct mhi_ring *ring,
						void **assigned_addr)
{
	return delete_element(ring, &ring->rp, &ring->wp, assigned_addr);
}

/**
 * delete_element - Moves the read pointer of the transfer ring to
 * the next element of the transfer ring,
 *
 * ring location of local ring data structure
 * @rp ring read pointer
 * @wp ring write pointer
 * @assigned_addr location of the element just deleted
 */
enum MHI_STATUS delete_element(struct mhi_ring *ring, void **rp,
			void **wp, void **assigned_addr)
<<<<<<< HEAD
=======
/**
*@brief Moves the read pointer of the transfer ring to
*the next element of the transfer ring,
*	as the element has been consumed by the mhi client.
*
*@param[in ] ring location of local register set for the ring
*@param[out] assigned_addr address returned to the calee with element
*/
MHI_STATUS delete_element(mhi_ring *ring, void * volatile *rp,
			void * volatile *wp, void **assigned_addr)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
{
	uintptr_t d_wp = 0, d_rp = 0, ring_size = 0;

	if (0 == ring->el_size || NULL == ring ||
		NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}
	ring_size = ring->len / ring->el_size;

	if (MHI_STATUS_SUCCESS != get_element_index(ring, *rp, &d_rp)) {
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != get_element_index(ring, *wp, &d_wp)) {
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
		return MHI_STATUS_ERROR;
	}
	if (d_wp == d_rp) {
		mhi_log(MHI_MSG_VERBOSE, "Ring 0x%lX is empty\n",
				(uintptr_t)ring->base);
		if (NULL != assigned_addr)
			*assigned_addr = NULL;
		return MHI_STATUS_RING_EMPTY;
	}

	if (NULL != assigned_addr)
		*assigned_addr = (void *)ring->rp;

	*rp = (void *)(((d_rp + 1) % ring_size) * ring->el_size +
						(uintptr_t)ring->base);

	return MHI_STATUS_SUCCESS;
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
int mhi_get_free_desc(struct mhi_client_handle *client_handle)
{
	u32 chan = client_handle->chan;
	struct mhi_device_ctxt *ctxt = client_handle->mhi_dev_ctxt;
	if (NULL == client_handle || MHI_HANDLE_MAGIC != client_handle->magic)
		return -EINVAL;
	return get_nr_avail_ring_elements(&ctxt->mhi_local_chan_ctxt[chan]);
}
EXPORT_SYMBOL(mhi_get_free_desc);

int get_nr_avail_ring_elements(struct mhi_ring *ring)
{
	u32 nr_el = 0;
	uintptr_t ring_size = 0;
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	ring_size = ring->len / ring->el_size;
	ret_val = get_nr_enclosed_el(ring, ring->rp, ring->wp, &nr_el);
	if (ret_val != MHI_STATUS_SUCCESS) {
		mhi_log(MHI_MSG_ERROR,
			"Failed to get enclosed el ret %d.\n", ret_val);
		return 0;
	}
	return ring_size - nr_el - 1;
}

enum MHI_STATUS get_nr_enclosed_el(struct mhi_ring *ring, void *rp,
						void *wp, u32 *nr_el)
<<<<<<< HEAD
=======
/**
 * @brief Returns the number of free ring elements.
 *
 * @param[in ]	ring	location of local register set for the ring
 * @param[out]	nr_of_avail_trbs returns the number of free elements
 */
int get_free_trbs(mhi_client_handle *client_handle)
{
	u32 chan = client_handle->chan;
	u32 available_ring_els = 0;
	mhi_device_ctxt *ctxt = client_handle->mhi_dev_ctxt;
	available_ring_els =
		get_nr_avail_ring_elements(&ctxt->mhi_local_chan_ctxt[chan]);
	return	available_ring_els;
}
int get_nr_avail_ring_elements(mhi_ring *ring)
{
	u32 nr_el = 0;
	uintptr_t ring_size = 0;
	ring_size = ring->len / ring->el_size;
	get_nr_enclosed_el(ring, ring->rp, ring->wp, &nr_el);
	return ring_size - nr_el - 1;
}

MHI_STATUS get_nr_enclosed_el(mhi_ring *ring, void *rp, void *wp, u32 *nr_el)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
{
	uintptr_t index_rp = 0;
	uintptr_t index_wp = 0;
	uintptr_t ring_size = 0;
	if (0 == ring->el_size || NULL == ring ||
		NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != get_element_index(ring, rp, &index_rp)) {
<<<<<<< HEAD
<<<<<<< HEAD
		mhi_log(MHI_MSG_CRITICAL, "Bad element index rp 0x%p.\n", rp);
=======
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		mhi_log(MHI_MSG_CRITICAL, "Bad element index rp 0x%p.\n", rp);
>>>>>>> 2617302... source
		return MHI_STATUS_ERROR;
	}

	if (MHI_STATUS_SUCCESS != get_element_index(ring, wp, &index_wp)) {
<<<<<<< HEAD
<<<<<<< HEAD
		mhi_log(MHI_MSG_CRITICAL, "Bad element index wp 0x%p.\n", wp);
=======
		mhi_log(MHI_MSG_CRITICAL, "Bad element index.\n");
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
		mhi_log(MHI_MSG_CRITICAL, "Bad element index wp 0x%p.\n", wp);
>>>>>>> 2617302... source
		return MHI_STATUS_ERROR;
	}
	ring_size = ring->len / ring->el_size;

	if (index_rp < index_wp)
		*nr_el = index_wp - index_rp;
	else if (index_rp > index_wp)
		*nr_el = ring_size - (index_rp - index_wp);
	else
		*nr_el = 0;
	return MHI_STATUS_SUCCESS;
}

<<<<<<< HEAD
<<<<<<< HEAD
enum MHI_STATUS get_element_index(struct mhi_ring *ring,
				void *address, uintptr_t *index)
=======
MHI_STATUS get_element_index(mhi_ring *ring, void *address, uintptr_t *index)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
enum MHI_STATUS get_element_index(struct mhi_ring *ring,
				void *address, uintptr_t *index)
>>>>>>> 2617302... source
{
	if (MHI_STATUS_SUCCESS != validate_ring_el_addr(ring,
							(uintptr_t)address))
		return MHI_STATUS_ERROR;
	*index = ((uintptr_t)address - (uintptr_t)ring->base) / ring->el_size;
	return MHI_STATUS_SUCCESS;
}

<<<<<<< HEAD
<<<<<<< HEAD
enum MHI_STATUS get_element_addr(struct mhi_ring *ring,
				uintptr_t index, void **address)
=======
MHI_STATUS get_element_addr(mhi_ring *ring, uintptr_t index, void **address)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
enum MHI_STATUS get_element_addr(struct mhi_ring *ring,
				uintptr_t index, void **address)
>>>>>>> 2617302... source
{
	uintptr_t ring_size = 0;
	if (NULL == ring || NULL == address)
		return MHI_STATUS_ERROR;
	ring_size = ring->len / ring->el_size;
	*address = (void *)((uintptr_t)ring->base +
			(index % ring_size) * ring->el_size);
	return MHI_STATUS_SUCCESS;
}
