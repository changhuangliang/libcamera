/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * file.cpp - File I/O operations
 */

#include "file.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"

/**
 * \file file.h
 * \brief File I/O operations
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(File);

/**
 * \class File
 * \brief Interface for I/O operations on files
 *
 * The File class provides an interface to perform I/O operations on files. It
 * wraps opening, closing and mapping files in memory, and handles the cleaning
 * of allocated resources.
 *
 * File instances are usually constructed with a file name, but the name can be
 * set later through the setFileName() function. Instances are not automatically
 * opened when constructed, and shall be opened explictly with open().
 *
 * Files can be mapped to the process memory with map(). Mapped regions can be
 * unmapped manually with munmap(), and are automatically unmapped when the File
 * is destroyed or when it is used to reference another file with setFileName().
 */

/**
 * \enum File::MapFlag
 * \brief Flags for the File::map() function
 * \var File::MapNoOption
 * \brief No option (used as default value)
 * \var File::MapPrivate
 * \brief The memory region is mapped as private, changes are not reflected in
 * the file constents
 */

/**
 * \enum File::OpenMode
 * \brief Mode in which a file is opened
 * \var File::NotOpen
 * \brief The file is not open
 * \var File::ReadOnly
 * \brief The file is open for reading
 * \var File::WriteOnly
 * \brief The file is open for writing
 * \var File::ReadWrite
 * \brief The file is open for reading and writing
 */

/**
 * \brief Construct a File to represent the file \a name
 * \param[in] name The file name
 *
 * Upon construction the File object is closed and shall be opened with open()
 * before performing I/O operations.
 */
File::File(const std::string &name)
	: name_(name), fd_(-1), mode_(NotOpen), error_(0)
{
}

/**
 * \brief Construct a File without an associated name
 *
 * Before being used for any purpose, the file name shall be set with
 * setFileName().
 */
File::File()
	: fd_(-1), mode_(NotOpen), error_(0)
{
}

/**
 * \brief Destroy a File instance
 *
 * Any memory mapping associated with the File is unmapped, and the File is
 * closed if it is open.
 */
File::~File()
{
	unmapAll();
	close();
}

/**
 * \fn const std::string &File::fileName() const
 * \brief Retrieve the file name
 * \return The file name
 */

/**
 * \brief Set the name of the file
 * \param[in] name The name of the file
 *
 * The \a name can contain an absolute path, a relative path or no path at all.
 * Calling this function on an open file results in undefined behaviour.
 *
 * Any memory mapping associated with the File is unmapped.
 */
void File::setFileName(const std::string &name)
{
	if (isOpen()) {
		LOG(File, Error)
			<< "Can't set file name on already open file " << name_;
		return;
	}

	unmapAll();

	name_ = name;
}

/**
 * \brief Check if the file specified by fileName() exists
 *
 * This function checks if the file specified by fileName() exists. The File
 * instance doesn't need to be open to check for file existence, and this
 * function may return false even if the file is open, if it was deleted from
 * the file system.
 *
 * \return True if the the file exists, false otherwise
 */
bool File::exists() const
{
	return exists(name_);
}

/**
 * \brief Open the file in the given mode
 * \param[in] mode The open mode
 *
 * This function opens the file specified by fileName() in \a mode. If the file
 * doesn't exist and the mode is WriteOnly or ReadWrite, this
 * function will attempt to create the file.
 *
 * The error() status is updated.
 *
 * \return True on success, false otherwise
 */
bool File::open(File::OpenMode mode)
{
	if (isOpen()) {
		LOG(File, Error) << "File " << name_ << " is already open";
		return false;
	}

	int flags = (mode & ReadWrite) - 1;

	fd_ = ::open(name_.c_str(), flags);
	if (fd_ < 0) {
		error_ = -errno;
		return false;
	}

	mode_ = mode;
	error_ = 0;
	return true;
}

/**
 * \fn bool File::isOpen() const
 * \brief Check if the file is open
 * \return True if the file is open, false otherwise
 */

/**
 * \fn OpenMode File::openMode() const
 * \brief Retrieve the file open mode
 * \return The file open mode
 */

/**
 * \brief Close the file
 *
 * This function closes the File. If the File is not open, it performs no
 * operation. Memory mappings created with map() are not destroyed when the
 * file is closed.
 */
void File::close()
{
	if (fd_ == -1)
		return;

	::close(fd_);
	fd_ = -1;
	mode_ = NotOpen;
}

/**
 * \fn int File::error() const
 * \brief Retrieve the file error status
 *
 * This function retrieves the error status from the last file open or I/O
 * operation. The error status is a negative number as defined by errno.h. If
 * no error occurred, this function returns 0.
 *
 * \return The file error status
 */

/**
 * \brief Retrieve the file size
 *
 * This function retrieves the size of the file on the filesystem. The File
 * instance shall be open to retrieve its size. The error() status is not
 * modified, error codes are returned directly on failure.
 *
 * \return The file size in bytes on success, or a negative error code otherwise
 */
ssize_t File::size() const
{
	if (!isOpen())
		return -EINVAL;

	struct stat st;
	int ret = fstat(fd_, &st);
	if (ret < 0)
		return -errno;

	return st.st_size;
}

/**
 * \brief Map a region of the file in the process memory
 * \param[in] offset The region offset within the file
 * \param[in] size The region sise
 * \param[in] flags The mapping flags
 *
 * This function maps a region of \a size bytes of the file starting at \a
 * offset into the process memory. The File instance shall be open, but may be
 * closed after mapping the region. Mappings stay valid when the File is
 * closed, and are destroyed automatically when the File is deleted.
 *
 * If \a size is a negative value, this function maps the region starting at \a
 * offset until the end of the file.
 *
 * The mapping memory protection is controlled by the file open mode, unless \a
 * flags contains MapPrivate in which case the region is mapped in read/write
 * mode.
 *
 * The error() status is updated.
 *
 * \return The mapped memory on success, or an empty span otherwise
 */
Span<uint8_t> File::map(off_t offset, ssize_t size, enum File::MapFlag flags)
{
	if (!isOpen()) {
		error_ = -EBADF;
		return {};
	}

	if (size < 0) {
		size = File::size();
		if (size < 0) {
			error_ = size;
			return {};
		}

		size -= offset;
	}

	int mmapFlags = flags & MapPrivate ? MAP_PRIVATE : MAP_SHARED;

	int prot = 0;
	if (mode_ & ReadOnly)
		prot |= PROT_READ;
	if (mode_ & WriteOnly)
		prot |= PROT_WRITE;
	if (flags & MapPrivate)
		prot |= PROT_WRITE;

	void *map = mmap(NULL, size, prot, mmapFlags, fd_, offset);
	if (map == MAP_FAILED) {
		error_ = -errno;
		return {};
	}

	maps_.emplace(map, size);

	error_ = 0;
	return { static_cast<uint8_t *>(map), static_cast<size_t>(size) };
}

/**
 * \brief Unmap a region mapped with map()
 * \param[in] addr The region address
 *
 * The error() status is updated.
 *
 * \return True on success, or false if an error occurs
 */
bool File::unmap(uint8_t *addr)
{
	auto iter = maps_.find(static_cast<void *>(addr));
	if (iter == maps_.end()) {
		error_ = -ENOENT;
		return false;
	}

	int ret = munmap(addr, iter->second);
	if (ret < 0) {
		error_ = -errno;
		return false;
	}

	maps_.erase(iter);

	error_ = 0;
	return true;
}

void File::unmapAll()
{
	for (const auto &map : maps_)
		munmap(map.first, map.second);

	maps_.clear();
}

/**
 * \brief Check if the file specified by \a name exists
 * \param[in] name The file name
 * \return True if the file exists, false otherwise
 */
bool File::exists(const std::string &name)
{
	struct stat st;
	int ret = stat(name.c_str(), &st);
	if (ret < 0)
		return false;

	return true;
}

} /* namespace libcamera */
