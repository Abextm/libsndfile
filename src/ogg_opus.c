/*
** Copyright (C) 2013-2016 Erik de Castro Lopo <erikd@mega-nerd.com>
**
** This program is free software ; you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation ; either version 2.1 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY ; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with this program ; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/


#include "sfconfig.h"

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#if HAVE_UNISTD_H
#include <unistd.h>
#else
#include "sf_unistd.h"
#endif

#include "sndfile.h"
#include "sfendian.h"
#include "common.h"

#if HAVE_EXTERNAL_XIPH_LIBS

#include <ogg/ogg.h>

#include "ogg.h"
#include <opus_multistream.h>

/* We use multiple ways to keep track of time here:
   Samples: 1 sample of data
	 Frames: 1 sample of data, per channel
	 Granules: 1 sample of data, per channel at 48kHz
*/
typedef struct
{
	/* How many granules to skip when starting to decode. */
	int32_t preskip ; 
	/* Opus decoder */
	OpusMSDecoder *dec ;
	/* Channel mapping given to the encoder.
	   Keep it so we can free it later */
	unsigned char *channel_mapping ;
	/* PCM data directly out of opus decode */
	struct {
		float *ptr ;
		/* ptr[0] .. ptr[start] .. ptr[start+len] .. ptr[capacity-1] */
		sf_count_t start, len, capacity ;
	} pcm ;
	/* Where decoding last started (file bytes)*/
	sf_count_t last_start ;
	/* Number of samples decoded since last start */
	sf_count_t samples_decoded ;
	/* Current position. This is set so 0 is the first sample after,
	   the preskip*/
	sf_count_t current_sample ;
	/* How many granules are missing from the front of the stream.
     Not filled when not seekable*/
	sf_count_t pre_roll;
} OPUS_PRIVATE ;


static int	ogg_opus_close (SF_PRIVATE *psf) ;
static int	ogg_opus_read_header (SF_PRIVATE *psf) ;
static sf_count_t	ogg_opus_seek (SF_PRIVATE *psf, int mode, sf_count_t offset) ;

static sf_count_t	ogg_opus_granule_to_frames (SF_PRIVATE *psf, sf_count_t granule) ;
static sf_count_t	ogg_opus_frames_to_samples (SF_PRIVATE *psf, sf_count_t frames) ;
static sf_count_t	ogg_opus_samples_to_frames (SF_PRIVATE *psf, sf_count_t samples) ;

static int	ogg_opus_convert_error (SF_PRIVATE *psf, int error) ;
static int	ogg_opus_read_packet (SF_PRIVATE *psf, int init) ;

typedef void	convert_func (SF_PRIVATE *, void *, sf_count_t, float *, sf_count_t, sf_count_t) ;
static sf_count_t	ogg_opus_read_sample (SF_PRIVATE *psf, void *ptr, sf_count_t lens, convert_func *write) ;
static void	opus_rnull (SF_PRIVATE *psf, void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len) ;

static sf_count_t	ogg_opus_read_s (SF_PRIVATE *psf, short *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_i (SF_PRIVATE *psf, int *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_f (SF_PRIVATE *psf, float *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_d (SF_PRIVATE *psf, double *ptr, sf_count_t len) ;

int
ogg_opus_open (SF_PRIVATE *psf)
{
	OGG_PRIVATE* odata = psf->container_data ;
	OPUS_PRIVATE* oodata = calloc (1, sizeof (OPUS_PRIVATE)) ;
	int	error = 0 ;

	if (odata == NULL)
	{
		psf_log_printf (psf, "%s : odata is NULL???\n", __func__) ;
		return SFE_INTERNAL ;
	}

	psf->codec_data = oodata ;
	if (oodata == NULL)
	{
		return SFE_MALLOC_FAILED ;
	}

	if (psf->file.mode == SFM_RDWR)
	{
		return SFE_BAD_MODE_RW ;
	}
		
	psf->sf.format = SF_FORMAT_OGG | SF_FORMAT_OPUS;
	psf->sf.sections=1;

	if (psf->file.mode == SFM_READ)
	{	/* There was some data that was consumed trying to figure out the file type
		   Feed it in and init the ogg stuff. */
		ogg_sync_init (&odata->osync) ;
		char *buffer = ogg_sync_buffer (&odata->osync, psf->header.indx) ;
		memcpy (buffer, psf->header.ptr, psf->header.indx) ;
		if (ogg_sync_wrote (&odata->osync, psf->header.indx)!=0)
		{
			return SFE_MALFORMED_FILE;
		}

		/* Read OpusHead and OpusTags */
		if ((error = ogg_opus_read_header (psf)))
		{
			return error ;
		}

		if (psf->sf.seekable)
		{
			oodata->last_start = psf_fseek (psf, 0, SEEK_CUR) ;
		}

		/* Opus likes to have a warm decoder, skip some stuff if requested */
		ogg_opus_read_sample (psf, NULL,
			ogg_opus_frames_to_samples (psf, ogg_opus_granule_to_frames (psf, oodata->preskip)),
			opus_rnull) ;

		psf->read_short  = ogg_opus_read_s ;
		psf->read_int    = ogg_opus_read_i ;
		psf->read_float  = ogg_opus_read_f ;
		psf->read_double = ogg_opus_read_d ;
		if (psf->sf.seekable)
		{
			sf_count_t frames = 0 ;
			for(;;)
			{
				error = ogg_opus_read_packet (psf, 0) ;
				if (error!=0)
				{
					return error ;
				}
				frames += opus_packet_get_nb_frames (odata->opacket.packet, odata->opacket.bytes)
					*opus_packet_get_samples_per_frame (odata->opacket.packet, psf->sf.samplerate) ;
				if (odata->opacket.granulepos>=0)
				{
					oodata->pre_roll = ogg_opus_granule_to_frames (psf, odata->opacket.granulepos) - frames;
					break ;
				}
			}
			sf_count_t pos = psf_fseek (psf, 0, SEEK_END) ;
			for(;;)
			{/* Seek from the back until we get a page, extract its granule */
				if (pos<=0)
				{
					psf->sf.frames = SF_COUNT_MAX ;
					break ;
				}
				pos-=4096 ;
				if(pos<0)
				{
					pos=0;
				}
				psf_fseek(psf, pos, SEEK_SET) ;
				ogg_sync_reset (&odata->osync) ;
				error = ogg_opus_read_packet (psf, 1) ;
				if (error==0)
				{
					break ;
				}
			}
			for(;;)
			{
				if (error!=0)
				{
					break ;
				}
				psf->sf.frames = ogg_opus_granule_to_frames (psf, odata->opacket.granulepos)-oodata->pre_roll ;
				error = ogg_opus_read_packet (psf, 0);
			}
			oodata->current_sample = ogg_opus_frames_to_samples (psf, psf->sf.frames) ;
			pos = ogg_opus_seek (psf, 0, 0) ;
		}
		else
		{
			psf->sf.frames = SF_COUNT_MAX ;
		}
	}

	psf->codec_close = ogg_opus_close ;

	if (psf->file.mode == SFM_WRITE)
	{
#if 0
		/* Set the default oodata quality here. */
		vdata->quality = 0.4 ;

		psf->write_header	= ogg_opus_write_header ;
		psf->write_short	= ogg_opus_write_s ;
		psf->write_int		= ogg_opus_write_i ;
		psf->write_float	= ogg_opus_write_f ;
		psf->write_double	= ogg_opus_write_d ;
#endif

		psf->sf.frames = SF_COUNT_MAX ; /* Unknown really */
		psf->strings.flags = SF_STR_ALLOW_START ;
	}

	psf->seek = ogg_opus_seek ;
#if 0
	psf->command = ogg_opus_command ;
#endif

	/* FIXME, FIXME, FIXME : Hack these here for now and correct later. */
	psf->datalength = 1 ;
	psf->dataoffset = 0 ;
	/* End FIXME. */

	return 0 ;
} /* ogg_opus_open */

static int
ogg_opus_close (SF_PRIVATE *psf)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	if (odata==NULL || oodata==NULL)
	{
		return 0 ;
	}
	if (oodata->dec!=NULL)
	{
		opus_multistream_decoder_destroy (oodata->dec) ;
	}
	if (oodata->channel_mapping!=NULL)
	{
		free (oodata->channel_mapping) ;
	}
	if (oodata->pcm.ptr!=NULL)
	{
		free (oodata->pcm.ptr) ;
	}
	return 0 ;
} /* ogg_opus_close */


/* puts a packet in odata->packet  */
static int
ogg_opus_read_packet (SF_PRIVATE *psf, int init)
{
	static const int CHUNK_SIZE = 16384;
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	int error = 0 ;
	for (int i=0;i<100;i++)
	{
		if (!init)
		{
			error = ogg_stream_packetout (&odata->ostream, &odata->opacket) ;
			if (error == 1) /* got a packet */
			{
				return 0 ;
			}
		}
		error = ogg_sync_pageout (&odata->osync, &odata->opage) ;
		if (error == 1)
		{
			if (init)
			{ /* reset the stream */
				ogg_stream_reset (&odata->ostream) ;
				error=ogg_stream_init (&odata->ostream, ogg_page_serialno (&odata->opage)) ;
				if (error!=0)
				{
					break ;
				}
				init=0;
			}
			ogg_stream_pagein (&odata->ostream, &odata->opage) ;
			continue ;
		}
		/* feed more data in */
		char *buffer = ogg_sync_buffer (&odata->osync, CHUNK_SIZE) ;
		size_t read = psf_fread (buffer, 1, CHUNK_SIZE, psf) ;
		if (read<=0)
		{
			return SFE_END_OF_FILE ;
		}
		error = ogg_sync_wrote (&odata->osync, read) ;
		if (error!=0)
		{
			break ;
		}
	}

	psf_log_printf (psf, "Input is not a valid Ogg bitstream.\n") ;
	return SFE_MALFORMED_FILE ;
} /* ogg_opus_read_packet */

static int opus_valid_samplerates [] =
{
	48000,
	24000,
	16000,
	12000,
	8000,
} ;

enum
{
	R128_TRACK_GAIN = -1,
	R128_ALBUM_GAIN = -2
} ;

static struct {
	int type ;
	const char *key ;
	int len ;
} opus_tag_map [] = {
	#define I( TYPE , STR ) { TYPE , STR , sizeof( STR ) -1 }
	I (R128_TRACK_GAIN,    "R128_TRACK_GAIN=" ),
	I (R128_ALBUM_GAIN,    "R128_ALBUM_GAIN=" ),
	I (SF_STR_TITLE,       "TITLE="           ),
	I (SF_STR_COPYRIGHT,   "COPYRIGHT="       ),
	I (SF_STR_ARTIST,      "ARTIST="          ),
	I (SF_STR_COMMENT,     "DESCRIPTION="     ),
	I (SF_STR_DATE,        "DATE="            ),
	I (SF_STR_ALBUM,       "ALBUM="           ),
	I (SF_STR_LICENSE,     "LICENSE="         ),
	I (SF_STR_TRACKNUMBER, "TRACKNUMBER="     ),
	I (SF_STR_GENRE,       "GENRE="           ),
	#undef I
} ;

/* Unlike vorbis_read_header, this does not support being called multiple times. It WILL leak */
static int
ogg_opus_read_header (SF_PRIVATE * psf)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	int error = 0 ;
	int16_t gain = 0;
	/* OpusHead */
	{
		if((error=ogg_opus_read_packet (psf, 1)))
		{
			return error;
		}
		if (odata->opacket.bytes<19 || memcmp (odata->opacket.packet, "OpusHead", 8)!=0)
		{
			psf_log_printf (psf, "Error reading opus header packet.\n") ;
			return SFE_MALFORMED_FILE ;
		}
		unsigned char *body = odata->opacket.packet;
		/* Version */
		unsigned char version = body[8] ;
		if ((version&0xF0)!=0)
		{
			psf_log_printf (psf, "Opus version %d is not implemented.\n", (int)(body[8])) ;
			return SFE_UNIMPLEMENTED ;
		}
		int channels = body[9] ;
		psf->sf.channels = channels ;
		oodata->preskip = (uint16_t)psf_get_le16 (body, 10) ;
		/* The samplerate in the header is just what it was encoded as, and not neccesarily decodable.
		   This allows the samplerate to be set in the SF_INFO struct */
		int samplerate = psf->sf.samplerate ;
		if (samplerate==0)
		{
			samplerate = psf_get_le32 (body, 12) ;
		}
		psf->sf.samplerate=48000 ;
		for(int i=0; i<ARRAY_LEN(opus_valid_samplerates); i++)
		{
			if (samplerate==opus_valid_samplerates[i])
			{
				psf->sf.samplerate=samplerate ;
				break ;
			}
		}
		oodata->current_sample=-ogg_opus_frames_to_samples (psf, ogg_opus_granule_to_frames (psf, oodata->preskip)) ;

		gain = psf_get_le16 (body, 16) ;

		char channel_map_family = body[18] ;
		int streams ;
		int coupled_streams ;
		unsigned char * mapping = calloc (channels, 1) ;
		oodata->channel_mapping = mapping ;
		/* map family zero doesn't require a map. Everything else does */
		if (channel_map_family==0)
		{
			if (channels==1)
			{
				streams = 1 ;
				coupled_streams = 0 ;
				mapping[0] = 0 ;
			}
			else if (channels==2)
			{
				streams = 1 ;
				coupled_streams = 1 ;
				mapping[0] = 0 ;
				mapping[1] = 1 ;
			}
			else
			{
				psf_log_printf (psf, "Opus file has too many streams for mapping.\n") ;
				return SFE_MALFORMED_FILE ;
			}
		}
		else /* map family!=0 */
		{
			if (odata->opacket.bytes<21+channels){
				psf_log_printf (psf, "Error reading opus header packet.\n") ;
				return SFE_MALFORMED_FILE ;
			}
			streams = body[19] ;
			coupled_streams = body[20] ;
			memcpy (mapping, body+21, channels);
		}
		oodata->dec = opus_multistream_decoder_create (
			psf->sf.samplerate, channels,
			streams, coupled_streams,
			mapping, &error) ;
		if (error!=OPUS_OK) {
			return ogg_opus_convert_error (psf, error) ;
		}
	}
	psf_log_printf (psf, "Bitstream is %d channel, %d Hz\n", psf->sf.channels, psf->sf.samplerate) ;
	/* OpusTags */
	{
		if((error=ogg_opus_read_packet (psf, 0)))
		{
			return error;
		}
		sf_count_t tl = odata->opacket.bytes ;
		if (tl<12 || memcmp (odata->opacket.packet, "OpusTags", 8)!=0)
		{
			psf_log_printf (psf, "Error reading opus tags packet.\n") ;
			return SFE_MALFORMED_FILE ;
		}
		unsigned char *body = odata->opacket.packet;
		int ofs = 8 ;
		/* encoder string */
		{
			int len = psf_get_le32(body, ofs) ;
			ofs+=4 ;
			if (tl<ofs+len+4)
			{
				return SFE_MALFORMED_FILE ;
			}
			char *tb = calloc (len+1, 1) ;
			memcpy (tb, body+ofs, len) ;
			psf_store_string (psf, SF_STR_SOFTWARE, tb) ;
			free (tb) ;
			ofs+=len ;
		}
		int count = psf_get_le32(body, ofs) ;
		ofs+=4 ;
		/* Set of count 'KEY=VALUE' pairs */
		for (int i=0; i<count; i++)
		{
			if (tl<ofs+4)
			{
				return SFE_MALFORMED_FILE ;
			}
			int len = psf_get_le32(body, ofs) ;
			ofs+=4 ;
			if (tl<ofs+len)
			{
				return SFE_MALFORMED_FILE ;
			}
			int type=0 ;
			int klen=0 ;
			for (int ii=0; ii<ARRAY_LEN(opus_tag_map); ii++)
			{
				if (len>=opus_tag_map[ii].len && memcmp (opus_tag_map[ii].key, body+ofs, opus_tag_map[ii].len)==0)
				{
					type = opus_tag_map[ii].type ;
					klen = opus_tag_map[ii].len ;
					break ;
				}
			}
			int alen=len-klen ;
			if (type!=0)
			{
				/* Get the part after the key */
				char *tb = calloc (alen+1, 1) ;
				memcpy (tb, body+ofs+klen, alen) ;
				if (type<0)
				{
					gain += strtol(tb,NULL,10) ;
				}
				else
				{
					psf_store_string (psf, SF_STR_SOFTWARE, tb) ;
				}
				free (tb) ;
			}
			ofs+=len ;
		}
		
		//psf->strings.data[0].type=SF_STR_
		/* TODO: implement reading tags */
	}
	/* this survives a decoder reset, so don't save it anywhere */
	opus_multistream_decoder_ctl (oodata->dec, OPUS_SET_GAIN(gain));
	return 0 ;
} /* ogg_opus_read_header */

static sf_count_t
ogg_opus_granule_to_frames (SF_PRIVATE *psf, sf_count_t granule)
{
	/* granules are time in 48k samples/sec. Convert to our samplerate */
	return (granule*48)/(psf->sf.samplerate/1000) ;
}
static sf_count_t
ogg_opus_frames_to_samples (SF_PRIVATE *psf, sf_count_t frames)
{
	return frames*psf->sf.channels ;
}
static sf_count_t
ogg_opus_samples_to_frames (SF_PRIVATE *psf, sf_count_t samples)
{
	return samples/psf->sf.channels ;
}

static sf_count_t
ogg_opus_seek (SF_PRIVATE *psf, int UNUSED (filemode), sf_count_t newpos)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	sf_count_t orig_newpos = newpos;


	if (newpos<0)
	{
		psf->error = SFE_BAD_SEEK ;
		return -1 ;
	}
	
	/* Shoot for this many frames early. This reduces misses and allows opus to warm its encoder
	   (Maximum of requested preskip, or 250 ms) times 2 */
	sf_count_t skip = 12000 ; /* 250ms */
	if (skip<oodata->preskip)
	{
		skip = oodata->preskip ;
	}
	skip=ogg_opus_granule_to_frames (psf, 2*skip);

	sf_count_t current_frame = ogg_opus_samples_to_frames (psf, oodata->current_sample) ;
	/* Have to seek backwards or more than 2*skip forward. Otherwise just skip some samples */
	if (newpos<current_frame || newpos>(current_frame+(skip*2)))
	{
		/* absolute of where to seek to */
		sf_count_t newfipos = 0;
		/* Simple case: seek to start */
		if (newpos==0)
		{
			newfipos = 0;
		}
		else
		{
			/* current file offset */
			sf_count_t fipos = psf_fseek (psf, 0, SEEK_CUR) ;
			/* bytes/frame */
			float framesize = (float)(fipos-oodata->last_start)/(float)ogg_opus_samples_to_frames (psf, oodata->samples_decoded) ;
			newpos-=skip ;
			/* calculate approx bytes to move*/
			sf_count_t delta = (float)((ogg_opus_granule_to_frames (psf, odata->opacket.granulepos)-oodata->pre_roll)-newpos)*framesize ;
			newfipos = fipos - delta ;
		}
		for (;;)
		{
			if (newfipos<0)
			{
				newfipos = 0 ;
			}

			oodata->last_start = psf_fseek (psf, newfipos, SEEK_SET) ;
			oodata->pcm.start = 0 ;
			oodata->pcm.len = 0 ;
			oodata->samples_decoded=0 ;
			ogg_sync_reset (&odata->osync) ;
			opus_multistream_decoder_ctl (oodata->dec, OPUS_RESET_STATE) ;
			
			if(newfipos<=0)
			{
				/* We need to start decoding data immediatly */
				oodata->current_sample=-ogg_opus_frames_to_samples (psf, ogg_opus_granule_to_frames (psf, oodata->preskip)) ;
				break ;
			}
			else
			{
				/* We are not starting at the beginning.
				   Read an entire packet and get its granule to calc our actual position*/

				/* Find an audio packet */
				for (int init=1;;init=0){
					ogg_opus_read_packet (psf, init) ;
					if (odata->opacket.bytes>8 && (memcmp (odata->opacket.packet, "OpusHead", 8)==0 || memcmp (odata->opacket.packet, "OpusTags", 8)==0))
					{
						continue ;
					}
					break ;
				}
				/* read the whole page, so the granule is where we start decoding, */
				for (;ogg_stream_packetout (&odata->ostream, &odata->opacket)==1;) ;
				sf_count_t current_frame = ogg_opus_granule_to_frames (psf, odata->opacket.granulepos)-oodata->pre_roll ;
				oodata->current_sample=ogg_opus_frames_to_samples (psf, current_frame) ;

				if (current_frame<=newpos)
				{	/* We got it close enough */
					break ;
				}
				/* We screwed up. Go back farther */
				newfipos-=16384 ;
			}
		}
	}
	/* conusme samples up until target */
	sf_count_t consume = ogg_opus_frames_to_samples (psf, orig_newpos)-oodata->current_sample ;
	if (consume>0)
	{
		ogg_opus_read_sample (psf, NULL, consume, opus_rnull) ;
	}
	sf_count_t newloc = ogg_opus_samples_to_frames (psf, oodata->current_sample) ;
	return newloc ;
} /* ogg_opus_seek */

static int
ogg_opus_convert_error (SF_PRIVATE *psf, int error)
{
	if (psf!=NULL && error!=OPUS_OK)
	{
		psf_log_printf (psf, "Error reading opus data: %s.\n", opus_strerror (error)) ;
	}
	switch (error)
	{
		case OPUS_OK:
			return 0 ;
		case OPUS_INVALID_PACKET:
			return SFE_MALFORMED_FILE ;
		case OPUS_ALLOC_FAIL:
			return SFE_MALLOC_FAILED ;
		default:
			return SFE_UNIMPLEMENTED ;
	}
} /* ogg_opus_convert_error */

static sf_count_t
ogg_opus_read_sample (SF_PRIVATE *psf, void *ptr, sf_count_t lens, convert_func *write)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	int error = 0 ;
	sf_count_t read = 0 ;
	/* 120ms is maximum opus frame size */
	sf_count_t frame_size = (psf->sf.samplerate*120)/1000;
	if (oodata->pcm.ptr==NULL)
	{ 
		sf_count_t cap = psf->sf.channels * frame_size ;
		oodata->pcm.ptr = calloc (cap, sizeof (float)) ;
		oodata->pcm.capacity = cap ;
		oodata->pcm.start = 0 ;
		oodata->pcm.len = 0 ;
	}
	for (;;)
	{
		if (oodata->pcm.len<=0)
		{
			oodata->pcm.start=0 ;
			error = ogg_opus_read_packet (psf, 0) ;
			if (error==0 && odata->opacket.bytes>8 && (memcmp (odata->opacket.packet, "OpusHead", 8)==0 || memcmp (odata->opacket.packet, "OpusTags", 8)==0))
			{ /* skip heads if we get them */
				continue ;
			}
			if (error!=0)
			{	
				if (error==SFE_END_OF_FILE && odata->osync.fill==0)
				{/* expected eof */
					return read ;
				}
				psf_log_printf (psf, "Courrupt or truncated data in bitstream; ") ;
				return read ;
			}
			int ss = opus_multistream_decode_float (oodata->dec,
				odata->opacket.packet, odata->opacket.bytes,
				oodata->pcm.ptr, frame_size, 0 ) ;
			if (ss>0)
			{
				sf_count_t len = ogg_opus_frames_to_samples (psf, ss) ;
				/* end trimming */
				if (odata->opacket.e_o_s)
				{
					sf_count_t last_sample = ogg_opus_frames_to_samples (psf, ogg_opus_granule_to_frames (psf, odata->opacket.granulepos)-oodata->pre_roll) ;
					sf_count_t max_read = last_sample-oodata->current_sample;
					if (max_read>=0 && len>max_read)
					{
						len = max_read ;
					}
				}
				oodata->pcm.len = len ;
			}
			else
			{
				psf->error = ogg_opus_convert_error(psf, ss) ;
				return read;
			}
		}
		sf_count_t len = lens-read ;
		if (len>oodata->pcm.len)
		{
			len = oodata->pcm.len;
		}
		write (psf, ptr, read, oodata->pcm.ptr, oodata->pcm.start, len) ;
		oodata->pcm.start += len ;
		oodata->pcm.len -= len ;
		read += len ;
		oodata->samples_decoded += len ; 
		oodata->current_sample += len ; 
		if(odata->opacket.granulepos>=0){
		}
		if (read >= lens)
		{
			return read ;
		}
	}
} /* ogg_opus_read_sample */

static void
opus_rnull (SF_PRIVATE *UNUSED(psf), void *UNUSED(out), sf_count_t UNUSED(read), float *UNUSED(in), sf_count_t UNUSED(start), sf_count_t UNUSED(len))
{
} /* opus_rnull */

static void
opus_rshort (SF_PRIVATE *psf, void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	float mul = 0x7FFF;
	if (psf->float_int_mult)
	{
		mul = (1.f/psf->float_max)*mul ;
	}
	for (sf_count_t i=0;i<len;i++)
	{
		((short*)out)[i+read] = lrintf (in[i+start]*mul) ;
	}
} /* opus_short */

static void
opus_rint (SF_PRIVATE *psf, void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	float mul = 0x7FFFFFFF;
	if (psf->float_int_mult)
	{
		mul = (1.f/psf->float_max)*mul ;
	}
	for (sf_count_t i=0;i<len;i++)
	{
		((int*)out)[i+read]=lrintf (in[i+start]*mul) ;
	}
} /* opus_rint */

static void
opus_rfloat (SF_PRIVATE *UNUSED (psf), void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	for (sf_count_t i=0;i<len;i++)
	{
		((float*)out)[i+read]=in[i+start] ;
	}
} /* opus_rfloat */

static void
opus_rdouble (SF_PRIVATE *UNUSED (psf), void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	for (sf_count_t i=0;i<len;i++)
	{
		((double*)out)[i+read]=in[i+start] ;
	}
} /* opus_rdouble */

static sf_count_t
ogg_opus_read_s (SF_PRIVATE *psf, short *ptr, sf_count_t lens)
{	return ogg_opus_read_sample (psf, (void*) ptr, lens, opus_rshort) ;
} /* ogg_opus_read_s */

static sf_count_t
ogg_opus_read_i (SF_PRIVATE *psf, int *ptr, sf_count_t lens)
{	return ogg_opus_read_sample (psf, (void*) ptr, lens, opus_rint) ;
} /* ogg_opus_read_i */

static sf_count_t
ogg_opus_read_f (SF_PRIVATE *psf, float *ptr, sf_count_t lens)
{	return ogg_opus_read_sample (psf, (void*) ptr, lens, opus_rfloat) ;
} /* ogg_opus_read_f */

static sf_count_t
ogg_opus_read_d (SF_PRIVATE *psf, double *ptr, sf_count_t lens)
{	return ogg_opus_read_sample (psf, (void*) ptr, lens, opus_rdouble) ;
} /* ogg_opus_read_d */

#else /* HAVE_EXTERNAL_XIPH_LIBS */

int
ogg_opus_open (SF_PRIVATE *psf)
{
	psf_log_printf (psf, "This version of libsndfile was compiled without Ogg/Opus support.\n") ;
	return SFE_UNIMPLEMENTED ;
} /* ogg_opus_open */

#endif
