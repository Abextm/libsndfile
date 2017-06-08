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

typedef struct
{
	/* How many samples to skip when starting to decode.
	   This is decremented as samples are skipped*/
	int32_t preskip ; 
	/* Sample multiplier */
	float gain ;
	/* Opus decoder */
	OpusMSDecoder *dec ;
	/* Channel mapping given to the encoder */
	unsigned char *channel_mapping ;
	/* pcm data directly out of opus decode */
	struct {
		float *ptr ;
		/* ptr[0] .. ptr[start] .. ptr[start+len] .. ptr[capacity-1] */
		sf_count_t start, len, capacity ;
	} pcm ;

} OPUS_PRIVATE ;


static int	ogg_opus_read_packet(SF_PRIVATE *psf, int init);
static int	ogg_opus_read_header (SF_PRIVATE *psf, int log_data) ;
static int	ogg_opus_close (SF_PRIVATE *psf) ;
static int	ogg_opus_convert_error (int error) ;
static sf_count_t	ogg_opus_read_s (SF_PRIVATE *psf, short *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_i (SF_PRIVATE *psf, int *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_f (SF_PRIVATE *psf, float *ptr, sf_count_t len) ;
static sf_count_t	ogg_opus_read_d (SF_PRIVATE *psf, double *ptr, sf_count_t len) ;

int
ogg_opus_open (SF_PRIVATE *psf)
{
	OGG_PRIVATE* odata = psf->container_data ;
	OPUS_PRIVATE* oopus = calloc (1, sizeof (OPUS_PRIVATE)) ;
	int	error = 0 ;

	if (odata == NULL)
	{
		psf_log_printf (psf, "%s : odata is NULL???\n", __func__) ;
		return SFE_INTERNAL ;
	}

	psf->codec_data = oopus ;
	if (oopus == NULL)
	{
		return SFE_MALLOC_FAILED ;
	}

	if (psf->file.mode == SFM_RDWR)
	{
		return SFE_BAD_MODE_RW ;
	}
		
	psf->sf.format = SF_FORMAT_OGG | SF_FORMAT_OPUS;
	psf->sf.sections=1;
	psf->sf.seekable=0;/* TODO: seekable */

	if (psf->file.mode == SFM_READ)
	{	/* There was some data that was consumed trying to figure out the file type
		   Feed it in and init the ogg stuff at the same time. */
		ogg_sync_init (&odata->osync) ;
		char *buffer = ogg_sync_buffer (&odata->osync, psf->header.indx) ;
		memcpy (buffer, psf->header.ptr, psf->header.indx) ;
		if (ogg_sync_wrote (&odata->osync, psf->header.indx)!=0)
		{
			return SFE_MALFORMED_FILE;
		}

		if ((error = ogg_opus_read_header (psf,1)))
		{
			return error ;
		}

		psf->read_short  = ogg_opus_read_s ;
		psf->read_int    = ogg_opus_read_i ;
		psf->read_float  = ogg_opus_read_f ;
		psf->read_double = ogg_opus_read_d ;
		//TODO: after seekable
		psf->sf.frames   = SF_COUNT_MAX; /*ogg_opus_length (psf) ; */
	}

	psf->codec_close = ogg_opus_close ;

	if (psf->file.mode == SFM_WRITE)
	{
#if 0
		/* Set the default oopus quality here. */
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

#if 0
	psf->seek = ogg_opus_seek ;
	psf->command = ogg_opus_command ;
#endif

	/* FIXME, FIXME, FIXME : Hack these here for now and correct later. */

	psf->datalength = 1 ;
	psf->dataoffset = 0 ;
	/* End FIXME. */

	return error ;
} /* ogg_opus_open */


/* puts a packet in odata->packet  */
static int
ogg_opus_read_packet(SF_PRIVATE *psf, int init)
{
	static const int CHUNK_SIZE = 4096;
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	/* OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ; */
	int error = 0 ;
	for(int i=0;i<100;i++){
		if (!init){
			error = ogg_stream_packetout (&odata->ostream, &odata->opacket) ;
			if (error == 1)
			{
				psf_log_printf(psf,"shifting packet\n");
				return 0 ;
			}
		}
		error = ogg_sync_pageout (&odata->osync, &odata->opage) ;
		if (error == 1)
		{
			if (init)
			{
				ogg_stream_reset(&odata->ostream) ;
				error=ogg_stream_init (&odata->ostream, ogg_page_serialno(&odata->opage)) ;
				if (error!=0)
				{
					break ;
				}
				init=0;
			}
			ogg_stream_pagein (&odata->ostream, &odata->opage) ;
			continue ;
		}
		char *buffer = ogg_sync_buffer(&odata->osync,CHUNK_SIZE) ;
		size_t read = psf_fread (buffer, 1, CHUNK_SIZE, psf) ;
		error = ogg_sync_wrote(&odata->osync, read) ;
		if (read<=0)
		{
			return SFE_END_OF_FILE ;
		}
		if(error!=0) {
			break;
		}
	}

	psf_log_printf (psf, "Input is not a valid Ogg bitstream.\n") ;
	return SFE_MALFORMED_FILE ;
} /* ogg_opus_read_packet */

static int
ogg_opus_read_header (SF_PRIVATE * psf, int log_data)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	int error = 0 ;

	/* OpusHead */
	{
		if((error=ogg_opus_read_packet (psf,1)))
		{
			return error;
		}
		if (odata->opacket.bytes<19 || memcmp(odata->opacket.packet,"OpusHead",8)!=0)
		{
			psf_log_printf (psf, "Error reading opus header packet.\n") ;
			return SFE_MALFORMED_FILE ;
		}
		unsigned char *body = odata->opacket.packet;
		/* Version */
		if (body[8]!=1)
		{
			psf_log_printf (psf, "Opus version %d is not implemented.\n ",(int)(body[8])) ;
			return SFE_UNIMPLEMENTED ;
		}
		int channels = psf->sf.channels = body[9] ;
		oodata->preskip = body[10] | body[11]<<8 ;
		psf->sf.samplerate = body[12] | (body[13]<<8) | (body[14]<<16) | (body[15]<<24) ;

		float raw_gain = body[16] | (body[17]<<8) ; //Q7.8
		oodata->gain = powf(10,raw_gain/(20.f*256.f)) ;

		char channel_map_family = body[18];
		int streams;
		int coupled_streams;
		unsigned char * mapping = oodata->channel_mapping = calloc(channels, 1) ;
		if (channel_map_family==0)
		{ // 1 stream, mono or stereo
			if (channels==1)
			{
				streams = 1 ;
				coupled_streams = 0 ;
				mapping[0] = 0 ;
			}
			else if(channels==2)
			{
				streams = 0 ;
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
		else 
		{
			if(odata->opacket.bytes<21+channels){
				psf_log_printf (psf, "Error reading opus header packet.\n") ;
				return SFE_MALFORMED_FILE ;
			}
			streams = body[19] ;
			coupled_streams = body[20] ;
			memcpy (mapping, body+21, channels);
		}
		oodata->dec = opus_multistream_decoder_create(psf->sf.samplerate, psf->sf.channels, streams, coupled_streams, mapping, &error) ;
		if(error!=OPUS_OK) {
			return ogg_opus_convert_error(error) ;
		}
	}
	psf_log_printf (psf, "Bitstream is %d channel, %d Hz\n",psf->sf.channels, psf->sf.samplerate) ;
	/* OpusTags */
	{
		if((error=ogg_opus_read_packet (psf,0)))
		{
			return error;
		}
		if (odata->opacket.bytes<12 || memcmp(odata->opacket.packet,"OpusTags",8)!=0)
		{
			psf_log_printf (psf, "Error reading opus tags packet.\n") ;
			return SFE_MALFORMED_FILE ;
		}
		unsigned char *body = odata->opacket.packet;
		/* TODO: implement reading tags */
	}
	return 0 ;
} /* ogg_opus_read_header */

static int
ogg_opus_close (SF_PRIVATE *psf)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	if (odata==NULL || oodata==NULL)
	{
		return 0 ;
	}
	if(oodata->dec!=NULL)
	{
		opus_multistream_decoder_destroy(oodata->dec) ;
	}
	if(oodata->channel_mapping!=NULL)
	{
		free(oodata->channel_mapping) ;
	}
	if (oodata->pcm.ptr!=NULL)
	{
		free(oodata->pcm.ptr) ;
	}

	return 0 ;
} /* ogg_opus_close */

static int
ogg_opus_convert_error (int error)
{
	switch (error)
	{
		case OPUS_OK:
			return 0 ;
		case OPUS_INVALID_PACKET:
			return SFE_MALFORMED_FILE ;
		case OPUS_ALLOC_FAIL:
			return SFE_MALLOC_FAILED ;
		default:
			return SFE_INTERNAL ;
	}
}

typedef void convert_func (SF_PRIVATE *, void *, sf_count_t, float *, sf_count_t, sf_count_t) ;

static sf_count_t
ogg_opus_read_sample (SF_PRIVATE *psf, void *ptr, sf_count_t lens, convert_func *write)
{
	OGG_PRIVATE *odata = (OGG_PRIVATE *) psf->container_data ;
	OPUS_PRIVATE *oodata = (OPUS_PRIVATE *) psf->codec_data ;
	int error = 0 ;
	sf_count_t read = 0 ;
	sf_count_t frame_size = (psf->sf.samplerate*120)/1000;
	if (oodata->pcm.ptr==NULL)
	{ /* TODO: try to calculate how many samples we actually need
	     Currently this is just the maximum possible by spec */
		sf_count_t cap = psf->sf.channels * frame_size ;
		oodata->pcm.ptr = calloc(cap,sizeof(float)) ;
		oodata->pcm.capacity = cap ;
		oodata->pcm.start = 0 ;
		oodata->pcm.len = 0 ;
	}
	for (;;)
	{
		if (oodata->pcm.len<=0)
		{
			oodata->pcm.start=0 ;
			if ((error = ogg_opus_read_packet (psf, 0)))
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
			psf_log_printf(psf,"%d\n",ss);
			if (ss>0)
			{
				oodata->pcm.len = ss ;
			}
			else
			{
				psf_log_printf (psf, "Courrupt or truncated data in bitstream; ") ;
				return read;
			}
			sf_count_t end = oodata->pcm.start+oodata->pcm.len ;
			for (sf_count_t i = oodata->pcm.start;i<end;i++)
			{
				oodata->pcm.ptr[i]*=oodata->gain ;
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
		if (read >= lens)
		{
			return read ;
		}
	}
} /* ogg_opus_read_sample */

static void
opus_rshort (SF_PRIVATE *psf, void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	float mul = 0x7FFF;
	if (psf->float_int_mult)
	{
		mul = (1.f/psf->float_max)*mul ;
	}
	for(sf_count_t i=0;i<len;i++)
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
	for(sf_count_t i=0;i<len;i++)
	{
		((int*)out)[i+read]=lrintf(in[i+start]*mul) ;
	}
} /* opus_rint */

static void
opus_rfloat (SF_PRIVATE *UNUSED (psf), void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	for(sf_count_t i=0;i<len;i++)
	{
		((float*)out)[i+read]=in[i+start] ;
	}
} /* opus_rfloat */

static void
opus_rdouble (SF_PRIVATE *UNUSED (psf), void *out, sf_count_t read, float *in, sf_count_t start, sf_count_t len)
{
	for(sf_count_t i=0;i<len;i++)
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
