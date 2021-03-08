// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE
#pragma once

#include <assert.h>
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhunistd.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhstdlib.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/scanobject.h"

#if _WIN32
#define ftello _ftelli64
#define fseeko _fseeki64
#define ftruncate(f, p) _chsize(f, (long)p)
#undef off_t
#define off_t __int64
#endif

class ScanSerializer
{
public:

    virtual ~ScanSerializer() {}

    //! Will be called to notify when a new scan is to be serialized. Should
    //! return false if the serializer implementation is unable to accept
    //! another scan, else true
    virtual bool begin_write_scan(uint32_t scan_sequence_number) = 0;

    //! Will be called to notify when the current scan is complete
    virtual void end_write_scan(bool aborted) = 0;

    //! Will be called when a new scan data type is to be serialized,
    //! for instance scan_info, clutter_image, detectreport, etc
    //! the data_type_name will be suitable as a unique filename
    //! for this scanning session. Returns false if unable to accept data
    virtual bool begin_write_scan_data_type(const char* data_type_name) = 0;

    //! Will be called with scan data to be written; elsize * count bytes
    //! Must return false if the write failed
    virtual bool write_scan_data_type(const void* data, size_t elsize, uint32_t count) = 0;

    //! Will be called to notify when the current scan data type is complete
    virtual void end_write_scan_data_type(bool aborted) = 0;


    //! called to notify of scan read begin, should return false if the scan
    //! sequence number is not available
    virtual bool begin_read_scan(uint32_t scan_sequence_number) = 0;

    //! called to notify of scan end
    virtual void end_read_scan() = 0;

    //! Will be called when a new scan data type is to be deserialized,
    //! for instance scan_info, clutter_image, detectreport, etc.
    //! Must return the number of bytes that were serialized for this data type
    //! (aka the file size). Return 0 if the data type is not available
    virtual size_t begin_read_scan_data_type(const char* data_type_name) = 0;

    //! Will be called with scan data to be read; elsize * count bytes
    //! Must return false if the read failed
    virtual bool read_scan_data_type(void* data, size_t elsize, uint32_t count) = 0;

    //! Will be called to notify when the current scan data type is complete
    virtual void end_read_scan_data_type() = 0;
};


//! Example scan serializer which writes the scan data to a folder
class SessionFolderScanSerializer : public ScanSerializer
{
public:

    SessionFolderScanSerializer(const char* folder)
        : fp(NULL)
        , fname(NULL)
    {
        session_folder = strdup(folder);
        session_folder_len = strlen(folder);
    }

    virtual ~SessionFolderScanSerializer()
    {
        if (fp)
        {
            fclose(fp);
        }

        free((void*)session_folder);
        delete [] fname;
    }

    // folders do not need to do anything special about new scan notifications
    virtual bool begin_write_scan(uint32_t)
    {
#ifdef _WIN32
        CreateDirectory(session_folder, NULL);
#else
        mkdir(session_folder, 0770);
#endif
        return true;
    }
    // to be pendantic, we would delete all of the scan files here if the scan
    // write was aborted.. instead we depend on scan deserialize to be robust
    virtual void end_write_scan(bool)       {}
    virtual bool begin_read_scan(uint32_t)  { return true; }
    virtual void end_read_scan()            {}

    virtual bool begin_write_scan_data_type(const char* data_type_name)
    {
        delete [] fname;
        fname = new char[session_folder_len + strlen(data_type_name) + 2];

        strcpy(fname, session_folder);
        strcpy(fname + session_folder_len, "/");
        strcpy(fname + session_folder_len + 1, data_type_name);

        fp = fopen(fname, "wb");
        return fp != NULL;
    }

    virtual bool write_scan_data_type(const void* data, size_t elsize, uint32_t count)
    {
        return fp && fwrite(data, elsize, count, fp) == count;
    }

    virtual void end_write_scan_data_type(bool aborted)
    {
        if (fp)
        {
            fclose(fp);
            fp = NULL;
        }

        if (aborted)
        {
            unlink(fname);
        }

        delete [] fname;
        fname = NULL;
    }

    virtual size_t begin_read_scan_data_type(const char* data_type_name)
    {
        delete [] fname;
        fname = new char[session_folder_len + strlen(data_type_name) + 2];

        strcpy(fname, session_folder);
        strcpy(fname + session_folder_len, "/");
        strcpy(fname + session_folder_len + 1, data_type_name);

        fp = fopen(fname, "rb");
        if (fp)
        {
            fseek(fp, 0, SEEK_END);
            off_t size = ftello(fp);
            rewind(fp);

            return (size_t)size;
        }
        else
        {
            return 0;
        }
    }

    virtual bool read_scan_data_type(void* data, size_t elsize, uint32_t count)
    {
        return fp && fread(data, elsize, count, fp) == count;
    }

    virtual void end_read_scan_data_type()
    {
        if (fp)
        {
            fclose(fp);
            fp = NULL;
        }

        delete [] fname;
        fname = NULL;
    }

protected:

    FILE*       fp;
    char*       fname;
    const char* session_folder;
    size_t      session_folder_len;
};


//! This serializer class can be used to communicate scan data between threads or
//! processes via a shared memory resource.  A mutex should be used to protect
//! the reader and writer from simultaneously accessing it. The class treats the
//! memory buffer as a ring buffer with variable sized scans. If the reader falls
//! too far behind the writer, serialize (write) calls will fail and scans will
//! be dropped.
//!
//! At the start of the buffer a ScanTable of MAX_SCANS_IN_BUFFER entries is
//! maintained. The writer and reader have separate pointers in this table.
//! dtype_table_offset references into the buffer, where a DTypeTable was
//! created for that scan. This table describes the data types that were
//! serialized for this scan, up to MAX_DTYPES_IN_SCAN data types. Scan data
//! wraps at the end of the buffer
class MemorySerializer : public ScanSerializer
{
public:

    enum { MAX_SCANS_IN_BUFFER = 16 };
    enum { MAX_DTYPES_IN_SCAN = 16 };

    struct ScanTable
    {
        enum
        {
            IDLE     = 0,
            WRITING  = 1,
            COMPLETE = 2,
            READING  = 3
        } state;
        uint32_t  sequence_number;
        size_t    dtype_table_offset;
    }* scan_table;

    struct DTypeTable
    {
        enum { MAX_NAME_LEN = 64 };
        char      dtype_name[MAX_NAME_LEN];
        size_t    starting_offset;
        size_t    size_bytes;
    };

    MemorySerializer(char* buffer, size_t buffer_sizebytes)
        : scan_table((ScanTable*)buffer)
        , buf(buffer)
        , len(buffer_sizebytes)
        , write_cur_scan(0)
        , write_cur_dtype(0)
        , read_cur_scan(0)
        , read_cur_dtype(0)
    {
        read_ptr = write_ptr = sizeof(ScanTable) * MAX_SCANS_IN_BUFFER;
        for (uint32_t i = 0; i < MAX_SCANS_IN_BUFFER; i++)
        {
            scan_table[i].state = ScanTable::IDLE;
            scan_table[i].dtype_table_offset = read_ptr;
        }
    }

    virtual ~MemorySerializer() {}

    //! non-virtual method, special to the memory serializer. It informs the
    //! reader which scan_sequence_number should be next read, if it is not
    //! being informed of the scan_sequence_numbers being written.  Returns
    //! true if a scan is ready to be read and the scan_sequence_number output
    //! value was set.  Returns false if no scan is ready to be read.
    bool get_next_read_scan_sequence(uint32_t& scan_sequence_number)
    {
        if (scan_table[read_cur_scan].state == ScanTable::COMPLETE)
        {
            scan_sequence_number = scan_table[read_cur_scan].sequence_number;
            return true;
        }
        else
        {
            return false;
        }
    }

    virtual bool begin_write_scan(uint32_t scan_sequence_number)
    {
        if (scan_table[write_cur_scan].state != ScanTable::IDLE)
            return false;

        size_t read_scan_ptr = scan_table[read_cur_scan].dtype_table_offset;

        // check if write_ptr would pass read_scan_ptr
        if (write_ptr < read_scan_ptr &&
            write_ptr + sizeof(DTypeTable) * MAX_DTYPES_IN_SCAN >= read_scan_ptr)
            return false;

        if (write_ptr + sizeof(DTypeTable) * MAX_DTYPES_IN_SCAN >= len)
        {
            // wrap write_ptr
            write_ptr = sizeof(ScanTable) * MAX_SCANS_IN_BUFFER;
            // check again if write_ptr would pass read_scan_ptr
            if (write_ptr + sizeof(DTypeTable) * MAX_DTYPES_IN_SCAN >= read_scan_ptr)
                return false;
        }
        scan_table[write_cur_scan].state = ScanTable::WRITING;
        scan_table[write_cur_scan].sequence_number = scan_sequence_number;
        scan_table[write_cur_scan].dtype_table_offset = write_ptr;
        DTypeTable* dtable = (DTypeTable*)(buf + write_ptr);
        for (uint32_t i = 0; i < MAX_DTYPES_IN_SCAN; i++)
        {
            dtable[i].starting_offset = 0;
            dtable[i].size_bytes = 0;
        }
        write_ptr += sizeof(DTypeTable) * MAX_DTYPES_IN_SCAN;
        write_cur_dtype = 0;
        return true;
    }

    virtual void end_write_scan(bool aborted)
    {
        if (aborted)
        {
            // this scan never existed
            scan_table[write_cur_scan].state = ScanTable::IDLE;
            write_ptr = scan_table[write_cur_scan].dtype_table_offset;
        }
        else
        {
            scan_table[write_cur_scan].state = ScanTable::COMPLETE;
            write_cur_scan = (write_cur_scan + 1 == MAX_SCANS_IN_BUFFER) ? 0 : write_cur_scan + 1;
        }
    }

    virtual bool begin_write_scan_data_type(const char* data_type_name)
    {
        if (write_cur_dtype >= MAX_DTYPES_IN_SCAN)
        {
            return false;
        }
        if (strlen(data_type_name) >= DTypeTable::MAX_NAME_LEN)
        {
            return false;
        }

        DTypeTable* dtable = (DTypeTable*)(buf + scan_table[write_cur_scan].dtype_table_offset);
        strcpy(dtable[write_cur_dtype].dtype_name, data_type_name);
        dtable[write_cur_dtype].starting_offset = write_ptr;
        dtable[write_cur_dtype].size_bytes = 0;
        return true;
    }

    virtual bool write_scan_data_type(const void* data, size_t elsize, uint32_t count)
    {
        DTypeTable* dtable = (DTypeTable*)(buf + scan_table[write_cur_scan].dtype_table_offset);
        elsize *= count;

        size_t read_scan_ptr = scan_table[read_cur_scan].dtype_table_offset;
        const char* datac = reinterpret_cast<const char*>(data);

        // check if write would pass read_scan_ptr
        if (write_ptr < read_scan_ptr && write_ptr + elsize >= read_scan_ptr)
            return false;

        if (write_ptr + elsize >= len)
        {
            // must split into two writes. write to the end
            dtable[write_cur_dtype].size_bytes += len - write_ptr;
            memcpy(buf + write_ptr, datac, len - write_ptr);
            elsize -= len - write_ptr;
            datac  += len - write_ptr;

            // wrap the write ptr, then check read_scan_ptr again
            uint32_t new_write_ptr = sizeof(ScanTable) * MAX_SCANS_IN_BUFFER;
            if (new_write_ptr == read_scan_ptr || new_write_ptr + elsize >= read_scan_ptr)
                return false;

            write_ptr = new_write_ptr;
        }

        memcpy(buf + write_ptr, datac, elsize);
        write_ptr += elsize;

        dtable[write_cur_dtype].size_bytes += elsize;
        return true;
    }

    virtual void end_write_scan_data_type(bool aborted)
    {
        DTypeTable* dtable = (DTypeTable*)(buf + scan_table[write_cur_scan].dtype_table_offset);
        if (aborted)
        {
            // this data type never existed
            dtable[write_cur_dtype].starting_offset = 0;
            dtable[write_cur_dtype].size_bytes = 0;
        }
        else
        {
            write_cur_dtype++;
        }
    }


    virtual bool begin_read_scan(uint32_t scan_sequence_number)
    {
        size_t orig = read_cur_scan;

        while (scan_table[read_cur_scan].state == ScanTable::COMPLETE &&
               scan_table[read_cur_scan].sequence_number < scan_sequence_number)
        {
            // assume the reader does not want this scan, discard it
            scan_table[read_cur_scan].state = ScanTable::IDLE;
            read_cur_scan = (read_cur_scan + 1 == MAX_SCANS_IN_BUFFER) ? 0 : read_cur_scan + 1;
            if (read_cur_scan == orig)
                return false;
        }

        if (scan_table[read_cur_scan].state == ScanTable::COMPLETE &&
            scan_table[read_cur_scan].sequence_number == scan_sequence_number)
        {
            scan_table[read_cur_scan].state = ScanTable::READING;
            return true;
        }
        return false;
    }

    virtual void end_read_scan()
    {
        scan_table[read_cur_scan].state = ScanTable::IDLE;
        read_cur_scan = (read_cur_scan + 1 == MAX_SCANS_IN_BUFFER) ? 0 : read_cur_scan + 1;
    }

    virtual size_t begin_read_scan_data_type(const char* data_type_name)
    {
        DTypeTable* dtable = (DTypeTable*)(buf + scan_table[read_cur_scan].dtype_table_offset);
        for (read_cur_dtype = 0; read_cur_dtype < MAX_DTYPES_IN_SCAN; read_cur_dtype++)
        {
            if (!strcmp(data_type_name, dtable[read_cur_dtype].dtype_name))
            {
                read_ptr = dtable[read_cur_dtype].starting_offset;
                return dtable[read_cur_dtype].size_bytes;
            }
        }

        return 0;
    }

    virtual bool read_scan_data_type(void* data, size_t elsize, uint32_t count)
    {
        // our synchronisation ensures that the entire file and entire scan is
        // completely written prior to reading, so we assume we can return the
        // full amount.  The ScanObject is responsible for ensuring it does not
        // read beyond the returned file size
        elsize *= count;
        char* datac = reinterpret_cast<char*>(data);

        if (read_ptr + elsize >= len)
        {
            memcpy(datac, buf + read_ptr, len - read_ptr);
            datac  += len - read_ptr;
            elsize -= len - read_ptr;
            read_ptr = sizeof(ScanTable) * MAX_SCANS_IN_BUFFER;
        }

        memcpy(datac, buf + read_ptr, elsize);
        read_ptr += elsize;
        return true;
    }

    virtual void end_read_scan_data_type()
    {
        // nop
    }

protected:

    char*  buf;
    size_t len;

    size_t write_cur_scan;
    size_t write_cur_dtype;
    size_t write_ptr;

    size_t read_cur_scan;
    size_t read_cur_dtype;
    size_t read_ptr;
};


class ArchiveScanSerializer : public ScanSerializer
{
public:

    FILE*       fp;
    char*       file_path;
    uint32_t    file_serial_id;
    bool        write_mode;

    off_t       scan_start_pos;

    struct file_header
    {
        uint32_t magic_value;
        uint32_t file_format_version;
    };

    enum { MAGIC_VALUE = 0xABBAC0ED };
    enum { FILE_FORMAT_VERSION = 1 };

    struct scan_header
    {
        uint64_t next_scan_start_pos;
        uint32_t magic_value;
        uint32_t scan_sequence_number;
    } cur_scan_hdr;

    struct
    {
        char     dtype_name[64];
        uint64_t dtype_start_pos;
        uint64_t dtype_size_bytes;
    } cur_dtype_hdr;

    //! location and file name base of capture file, examples:
    //!    ArchiveScanSerializer("C:/storage/drive", true);
    //! will result in:
    //!    C:/storage/drive_archive_0.bin
    //! The _0 is a serial number, the archiver will bump this serial ID
    //! when the archive grows to a certain size
    ArchiveScanSerializer(const char* path, bool _write_mode)
        : fp(NULL)
        , file_path(NULL)
        , file_serial_id(0)
        , write_mode(_write_mode)
        , scan_start_pos(0)
    {
        if (!path) return;

        file_path = new char[strlen(path) + 32];

        sprintf(file_path, "%sarchive_%d.bin", path, file_serial_id);
        fp = fopen(file_path, write_mode ? "wb" : "rb");

        if (!fp)
        {
            printf("Unable to open archive file <%s>\n", file_path);
            return;
        }
        else if (write_mode)
        {
            file_header hdr;
            hdr.magic_value = MAGIC_VALUE;
            hdr.file_format_version = FILE_FORMAT_VERSION;
            fwrite(&hdr, sizeof(hdr), 1, fp);
        }
        else
        {
            file_header hdr;
            if (fread(&hdr, sizeof(hdr), 1, fp) != 1)
            {
                printf("Archive file does not have header\n");
                fclose(fp);
                fp = NULL;
                return;
            }
            if (hdr.magic_value != MAGIC_VALUE ||
                hdr.file_format_version != FILE_FORMAT_VERSION)
            {
                printf("Archive file does not have appropriate headers\n");
                fclose(fp);
                fp = NULL;
                return;
            }
        }

        scan_start_pos = ftello(fp);
        if (!write_mode)
        {
            cur_scan_hdr.next_scan_start_pos = scan_start_pos;
        }
    }


    bool is_ok()
    {
        return fp != NULL;
    }


    const char* get_archive_file_path() const
    {
        return file_path;
    }


    virtual ~ArchiveScanSerializer()
    {
        if (fp)
        {
            fclose(fp);
        }
        delete [] file_path;
    }


    //! This is a scan iterator that can be used with a read-only archive to
    //! extract all of the scans in the most efficient way possible.
    ScanObject* get_next_scan()
    {
        if (!fp || write_mode)
        {
            return NULL;
        }

        // TODO: check for archive overflow here, bump serial id and reopen

        if (fseeko(fp, cur_scan_hdr.next_scan_start_pos, SEEK_SET) < 0)
        {
            return NULL;
        }
        scan_header tmp;
        if (fread(&tmp, sizeof(tmp), 1, fp) == 1)
        {
            if (tmp.magic_value != MAGIC_VALUE)
            {
                printf("scan header appears invalid, aborting\n");
                return NULL;
            }
            return ScanObject::deserialize(*this, tmp.scan_sequence_number);
        }
        else
        {
            return NULL;
        }
    }

    //! returns the number of scans contained in a read-only archive
    uint32_t get_scan_count()
    {
        if (!fp || write_mode)
        {
            return 0;
        }

        // get archive size
        fseeko(fp, 0, SEEK_END);
        uint64_t total_size = ftello(fp);

        // start at the beginning of the file
        fseeko(fp, sizeof(file_header), SEEK_SET);

        uint32_t count = 0;

        scan_header tmp;
        do
        {
            if (fread(&tmp, sizeof(tmp), 1, fp) == 1)
            {
                if (tmp.magic_value != MAGIC_VALUE)
                {
                    return count;
                }

                count++;

                // move to the next scan header
                fseeko(fp, tmp.next_scan_start_pos, SEEK_SET);
            }
            else
            {
                return count;
            }
        }
        while (tmp.next_scan_start_pos < total_size);

        return count;
    }


    struct ScanSummary
    {
        uint32_t scan_sequence_number;
        uint64_t scan_file_position;
    };

    //! returns summary of scans contained in a read-only archive, user must
    //! provide output storage for scan summaries. returns count of summaries
    //! which contain valid data
    uint32_t get_scan_summaries(ScanSummary* sum_buffer, uint32_t buffer_size)
    {
        if (!fp || write_mode)
        {
            return 0;
        }

        // get archive size
        fseeko(fp, 0, SEEK_END);
        uint64_t total_size = ftello(fp);

        // start at the beginning of the file
        fseeko(fp, sizeof(file_header), SEEK_SET);

        uint32_t count = 0;

        scan_header tmp;
        do
        {
            off_t curpos = ftello(fp);

            if (fread(&tmp, sizeof(tmp), 1, fp) == 1)
            {
                if (tmp.magic_value != MAGIC_VALUE)
                {
                    return count;
                }

                sum_buffer[count].scan_sequence_number = tmp.scan_sequence_number;
                sum_buffer[count].scan_file_position = curpos;
                count++;

                // move to the next scan header
                fseeko(fp, tmp.next_scan_start_pos, SEEK_SET);
            }
            else
            {
                return count;
            }
        }
        while (tmp.next_scan_start_pos < total_size);

        return count;
    }

    //! extract a scan from the archive (random access)
    ScanObject* get_scan_at_position(const ScanSummary& sum)
    {
        if (!fp || write_mode)
        {
            return NULL;
        }

        cur_scan_hdr.next_scan_start_pos = sum.scan_file_position;
        return ScanObject::deserialize(*this, sum.scan_sequence_number);
    }



    virtual bool begin_write_scan(uint32_t scan_sequence_number)
    {
        if (!fp || !write_mode) return false;

        // TODO: check for archive overflow here, bump serial id and reopen

        fseeko(fp, 0, SEEK_END);
        scan_start_pos = ftello(fp);

        cur_scan_hdr.scan_sequence_number = scan_sequence_number;
        cur_scan_hdr.magic_value = MAGIC_VALUE;
        cur_scan_hdr.next_scan_start_pos = (uint64_t)-1; // NULL
        return (fwrite(&cur_scan_hdr, sizeof(cur_scan_hdr), 1, fp) == 1);
    }


    virtual void end_write_scan(bool aborted)
    {
        if (!write_mode) return;

        if (aborted)
        {
            fseeko(fp, scan_start_pos, SEEK_SET);
            if (ftruncate(fileno(fp), scan_start_pos) < 0)
            {
                printf("Failed to truncate aborted scan\n");
            }
        }
        else
        {
            // record the offset of the next scan start in the just completed
            // scan's header
            cur_scan_hdr.next_scan_start_pos = ftello(fp);
            fseeko(fp, scan_start_pos, SEEK_SET);
            fwrite(&cur_scan_hdr, sizeof(cur_scan_hdr), 1, fp);
            fseeko(fp, cur_scan_hdr.next_scan_start_pos, SEEK_SET);
        }
    }


    virtual bool begin_write_scan_data_type(const char* data_type_name)
    {
        if (!write_mode)
        {
            return false;
        }

        assert(strlen(data_type_name) < sizeof(cur_dtype_hdr.dtype_name));
        strcpy(cur_dtype_hdr.dtype_name, data_type_name);
        cur_dtype_hdr.dtype_start_pos = ftello(fp) + sizeof(cur_dtype_hdr);
        cur_dtype_hdr.dtype_size_bytes = 0;
        return fwrite(&cur_dtype_hdr, sizeof(cur_dtype_hdr), 1, fp) == 1;
    }

    virtual bool write_scan_data_type(const void* data, size_t elsize, uint32_t count)
    {
        if (!write_mode)
        {
            return false;
        }

        return (fwrite(data, elsize, count, fp) == count);
    }

    virtual void end_write_scan_data_type(bool aborted)
    {
        if (!aborted)
        {
            // finish the current data type header by writing the length
            cur_dtype_hdr.dtype_size_bytes = ftello(fp) - cur_dtype_hdr.dtype_start_pos;
            fseeko(fp, cur_dtype_hdr.dtype_start_pos - sizeof(cur_dtype_hdr), SEEK_SET);
            fwrite(&cur_dtype_hdr, sizeof(cur_dtype_hdr), 1, fp);
            fseeko(fp, 0, SEEK_END);
        }
    }


    virtual bool begin_read_scan(uint32_t scan_sequence_number)
    {
        if (write_mode)
        {
            return false;
        }

        scan_start_pos = cur_scan_hdr.next_scan_start_pos;

        if (fseeko(fp, scan_start_pos, SEEK_SET) < 0)
        {
            return false;
        }
        if (fread(&cur_scan_hdr, sizeof(cur_scan_hdr), 1, fp) == 1)
        {
            // note - random access is not supported, users should be using
            // the archive's get_next_scan() method
            return cur_scan_hdr.scan_sequence_number == scan_sequence_number;
        }
        else
        {
            return false;
        }
    }


    virtual void end_read_scan()
    {
        // NOP
    }


    virtual size_t begin_read_scan_data_type(const char* data_type_name)
    {
        if (write_mode)
        {
            return 0;
        }

        if (fseeko(fp, scan_start_pos + sizeof(scan_header), SEEK_SET) < 0)
        {
            return 0;
        }
        while ((uint64_t)ftello(fp) < cur_scan_hdr.next_scan_start_pos)
        {
            if (fread(&cur_dtype_hdr, sizeof(cur_dtype_hdr), 1, fp) != 1)
            {
                return 0;
            }

            if (!strcmp(cur_dtype_hdr.dtype_name, data_type_name))
            {
                fseeko(fp, cur_dtype_hdr.dtype_start_pos, SEEK_SET);
                return cur_dtype_hdr.dtype_size_bytes;
            }
            else
            {
                // advance to next data type
                fseeko(fp, cur_dtype_hdr.dtype_start_pos + cur_dtype_hdr.dtype_size_bytes, SEEK_SET);
            }
        }

        return 0;
    }


    virtual bool read_scan_data_type(void* data, size_t elsize, uint32_t count)
    {
        if (write_mode)
        {
            return false;
        }

        return (fread(data, elsize, count, fp) == count);
    }


    virtual void end_read_scan_data_type()
    {
        // NOP
    }

    virtual void flush()
    {
        if(fp && write_mode)
        {
            fflush(fp);
        }
    }
};
