
#include "ssf_flash.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdatomic.h>
#include "debug.h"
#include "ssf_main.h"

typedef void (*flash_pageScanBlockFun_t)(const flash_blockInfo_t* blockInfo, void* context);


typedef struct {
	uintptr_t configFlashStart, configFlashEnd;
	uintptr_t firstEntry, lastEntry;
	size_t numPages;
	uintptr_t lastEntriesByType[FLASH_CONFIG_MAX_TYPES];
	uint16_t sequenceCounter;

	bool writeError;
	atomic_flag writeLock;
} _flashRuntime_t;

_flashRuntime_t self = {
	.configFlashStart = 0x0,
	.configFlashEnd = 0x0,
	.firstEntry = 0x0,
	.lastEntry = 0x0,
	.numPages = 0,
	.writeLock = ATOMIC_FLAG_INIT,
};

static uint16_t _nextSequenceNumber(const uint16_t seq)
{
	return seq + 1 + (seq == 0xFFFE);
}

static uintptr_t _blockAddrToPageNumber(const uintptr_t blockAddr)
{
	return ((blockAddr - self.configFlashStart)/FLASH_PAGE_SIZE);
}

static uintptr_t _blockAddrToPageAddr(const uintptr_t blockAddr)
{
	return (blockAddr/FLASH_PAGE_SIZE)*FLASH_PAGE_SIZE;
}

static bool _isValidBlockAddress(const uintptr_t blockAddr)
{
	return ((blockAddr >= self.configFlashStart) && (blockAddr + sizeof(flash_blockInfo_t) <= self.configFlashEnd));
}

static uintptr_t _blockEnd(const uintptr_t blockAddr)
{
	if (!_isValidBlockAddress(blockAddr))
		return 0;
	else
		return blockAddr + ((flash_blockInfo_t*)blockAddr)->numWords*FLASH_WORD_SIZE;
}

static uintptr_t _blockSize(const uintptr_t blockAddr)
{
	if (!_isValidBlockAddress(blockAddr))
		return 0;
	else
		return ((flash_blockInfo_t*)blockAddr)->numWords*FLASH_WORD_SIZE;
}



static uint16_t _sequenceNumberOfBlock(const uintptr_t blockAddr)
{
	if (!_isValidBlockAddress(blockAddr))
		return -1;
	else
		return ((flash_blockInfo_t*)blockAddr)->sequenceId;
}

static void _scanPageBlocks(const uintptr_t pageAddr, flash_pageScanBlockFun_t blockFun, void* blockFunContext)
{
	uintptr_t addr = pageAddr;
	while (addr + sizeof(flash_blockInfo_t) < pageAddr + FLASH_PAGE_SIZE)
	{
		const flash_blockInfo_t* blockInfo  = (const void*)addr;
		bool noSeq = blockInfo->sequenceId == 0xFFFF;
		bool noLen = blockInfo->numWords == 0xFF;
		bool noFlags = blockInfo->flags == 0xFF;

		bool isEmpty = noSeq && noLen && noFlags;

		// bool preserved = (0 != (blockInfo->flags & (1 << FLASH_CONFIG_FLAG_JOURNAL_BIT)));

		if (isEmpty)
			break;

		if (blockFun)
			blockFun(blockInfo, blockFunContext);

		if (noLen || (blockInfo->numWords == 0))
			break;

		addr += blockInfo->numWords*FLASH_WORD_SIZE;
	}
}



// this function just sets the block addr in the context array for blocks that have to be preserved
// it does not check the sequence number of the block to make sure its newer
// so it needs to be called starting at the beginning of the journal to find the last entry
static void _preserveScanBlockFun(const flash_blockInfo_t* blockInfo, void* context)
{
	uintptr_t* entries = context;

	size_t type = blockInfo->flags & (FLASH_CONFIG_MAX_TYPES - 1u);

	entries[type] = (uintptr_t)blockInfo;
}

// this collects the first and last sequence IDs on a page
static void _sequenceScanBlockFun(const flash_blockInfo_t* blockInfo, void* context)
{
	uintptr_t (*firstlast)[2] = context;
	uint16_t firstSeq = firstlast[0][1];
	uint16_t lastSeq = firstlast[1][1];

	if (firstSeq == 0xFFFF)
	{
		firstlast[0][0] = firstlast[1][0] = (uintptr_t)(const void*)blockInfo;
		firstlast[0][1] = firstlast[1][1] = blockInfo->sequenceId;
		return;
	}

	// next is increment of last, except 0xFFFF is skipped
	uint16_t nextSeq = _nextSequenceNumber(lastSeq);

	if (blockInfo->sequenceId == nextSeq)
	{
		firstlast[1][0] = (uintptr_t)(const void*)blockInfo;
		firstlast[1][1] = blockInfo->sequenceId;
	}

}

static bool _eraseConfigPages(size_t start, size_t count)
{
	FLASH_EraseInitTypeDef erase = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Page = (self.configFlashStart-FLASH_BASE)/FLASH_PAGE_SIZE + start,
		.NbPages = count,
		.Banks = FLASH_BANK_1,
	};
	// for some reason PG bit was set prior to calling this, so reset it
	// even though in theory it should only be set during write ops
	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
	// err_println("  pre-erase error 0x%08X SR 0x%08X.", HAL_FLASH_GetError(), FLASH->SR);
	HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &errpage);

	if (status != HAL_OK)
	{
		uint32_t error = HAL_FLASH_GetError();
		err_println("  erase failed with status 0x%08X, page %d, error 0x%08X.", status, errpage, error);
		err_println("  SR 0x%08X.", FLASH->SR);
		return false;
	}

	return true;
}


static void _configFlashInit(void)
{
	size_t freeBytes = (uintptr_t)(__configflash_end_addr - __configflash_firstPage_addr);

	dbg_println("Config Flash memory starts at 0x%08X", (uintptr_t)__configflash_firstPage_addr);
	dbg_println("Config Flash memory ends at   0x%08X", (uintptr_t)__configflash_end_addr);
	dbg_println("Config Flash memory contains  0x%08X (%u) bytes", freeBytes, freeBytes);

	size_t numFreePages = freeBytes/FLASH_PAGE_SIZE;
	dbg_println("                              0x%08X (%u) pages", numFreePages, numFreePages);

	self.configFlashStart = (uintptr_t)__configflash_firstPage_addr;
	self.configFlashEnd = (uintptr_t)__configflash_end_addr;
	self.numPages = numFreePages;
	self.sequenceCounter = -1;

	// unlock flash for writing
	{
		HAL_StatusTypeDef status = HAL_FLASH_Unlock();
		if (status != HAL_OK)
		{
			err_println("  failed to unlock flash.");
			atomic_flag_clear(&self.writeLock);
			return;
		}
	}

/*
	config flash has to be scanned to
		1. find beginning/end of entries

*/

	// find the an occupied block as a starting point
	for (size_t i = 0; i < self.numPages; ++i)
	{
		uintptr_t pageAddr = self.configFlashStart + i*FLASH_PAGE_SIZE;

		// first and last entry address and sequence numbers
		uintptr_t firstlast[2][2] = {{-1, -1}, {-1, -1}};
		_scanPageBlocks(pageAddr, _sequenceScanBlockFun, firstlast);


		if (firstlast[0][0] != -1)
		{
			self.firstEntry = firstlast[0][0];
			self.lastEntry = firstlast[1][0];
			break;
		}
	}

	// scan find extents of data if we found anything at all
	if (self.firstEntry != 0)
	{
		// begin with scanning forward to find end
		size_t startingPage = _blockAddrToPageNumber(self.firstEntry);
		for (size_t i = 1; i < self.numPages; ++i)
		{
			size_t page = (i + startingPage) % self.numPages;
			uintptr_t pageAddr = self.configFlashStart + page*FLASH_PAGE_SIZE;

			// first and last entry address and sequence numbers
			uintptr_t firstlast[2][2] = {{-1, -1}, {-1, -1}};
			_scanPageBlocks(pageAddr, _sequenceScanBlockFun, firstlast);


			if (firstlast[0][0] != -1)
			{
				uint16_t last = _sequenceNumberOfBlock(self.lastEntry);
				uint16_t blockFirst = firstlast[0][1];
				uint16_t logicalNext = _nextSequenceNumber(last);

				// if this block follows logically after the previous, we continue
				if (logicalNext == blockFirst)
				{
					self.lastEntry = firstlast[1][0];
				}
				else
				{
					// else we have a discontinuity
					break;
				}
			}
			else
			{
				// empty page, break
				break;
			}
		}

		// then scan backward to find beginning
		for (size_t i = 1; i < self.numPages; ++i)
		{
			// scanning backwards
			size_t page = (self.numPages + startingPage - i) % self.numPages;
			uintptr_t pageAddr = self.configFlashStart + page*FLASH_PAGE_SIZE;

			// first and last entry address and sequence numbers
			uintptr_t firstlast[2][2] = {{-1, -1}, {-1, -1}};
			_scanPageBlocks(pageAddr, _sequenceScanBlockFun, firstlast);


			if (firstlast[0][0] != -1)
			{
				uint16_t blockLast = firstlast[1][1];
				uint16_t first = _sequenceNumberOfBlock(self.firstEntry);
				uint16_t logicalNext = _nextSequenceNumber(blockLast);

				// if this block precedes the previous, we continue
				if (logicalNext == first)
				{
					self.firstEntry = firstlast[0][0];
				}
				else
				{
					// else we have a discontinuity
					break;
				}
			}
			else
			{
				// empty page, break
				break;
			}
		}

		// then scan for the last entry of each block type
		startingPage = _blockAddrToPageNumber(self.firstEntry);
		size_t endPage = _blockAddrToPageNumber(self.lastEntry);
		size_t pageCount = startingPage <= endPage ? endPage - startingPage : endPage - startingPage + self.numPages;
		
		// reset entries tables
		memset(self.lastEntriesByType, 0, sizeof(self.lastEntriesByType));

		for (size_t i = 0; i < pageCount; ++i)
		{
			size_t page = (i + startingPage) % self.numPages;
			uintptr_t pageAddr = self.configFlashStart + page*FLASH_PAGE_SIZE;

			_scanPageBlocks(pageAddr, _preserveScanBlockFun, self.lastEntriesByType);
		}

		// finally update sequence counter for next write
		self.sequenceCounter = _sequenceNumberOfBlock(self.lastEntry);
	}

	dbg_println("First journal entry at 0x%08X", (uintptr_t)self.firstEntry);
	dbg_println("Last journal entry at  0x%08X", (uintptr_t)self.lastEntry);

}

void ssf_flashInit(void)
{
	size_t freeBytes = (uintptr_t)(__freeflash_end_addr - __freeflash_firstPage_addr);

	dbg_println("Free Flash memory starts at 0x%08X", (uintptr_t)__freeflash_firstPage_addr);
	dbg_println("Free Flash memory ends at   0x%08X", (uintptr_t)__freeflash_end_addr);
	dbg_println("Free Flash memory contains  0x%08X (%u) bytes", freeBytes, freeBytes);

	size_t numFreePages = freeBytes/FLASH_PAGE_SIZE;
	dbg_println("                            0x%08X (%u) pages", numFreePages, numFreePages);

	_configFlashInit();
}

static size_t zmin(size_t a, size_t b)
{
	return a > b ? b : a;
}

static bool _writeBlock(flash_blockInfo_t blockInfo, uintptr_t data)
{
	// check if write error has been set, don't proceed
	if (self.writeError)
		return false;

	uintptr_t writeAddr = (self.lastEntry != 0) ? _blockEnd(self.lastEntry) : self.configFlashStart;

	size_t freeBytes = FLASH_PAGE_SIZE - (writeAddr - _blockAddrToPageAddr(writeAddr));
	// skip to next page if not enough space on this
	if (freeBytes < blockInfo.numWords*FLASH_WORD_SIZE)
	{
		// this assumes next page is free
		size_t writepage = (_blockAddrToPageNumber(writeAddr)+1) % self.numPages;
		assert(_blockAddrToPageNumber(self.firstEntry) != writepage);

		writeAddr = self.configFlashStart + FLASH_PAGE_SIZE*writepage;
	}

	// set sequenceId on blockInfo
	uint16_t sequence = _nextSequenceNumber(self.sequenceCounter);
	blockInfo.sequenceId = sequence;

	// write one word at a time, first blockInfo, then data
	uintptr_t blockAddr = (uintptr_t)&blockInfo;
	for (size_t i = 0; i < blockInfo.numWords; ++i)
	{
		uint64_t word = -1;
		size_t chunkOffset = 0;
		if (i*FLASH_WORD_SIZE < sizeof(flash_blockInfo_t))
		{
			chunkOffset = zmin(FLASH_PAGE_SIZE, sizeof(flash_blockInfo_t) - i*FLASH_WORD_SIZE);
			memcpy(&word, (const void*)(blockAddr + i*FLASH_WORD_SIZE), chunkOffset);
		}
		memcpy((void*)(((uintptr_t)&word) + chunkOffset), (const void*)(data + i*FLASH_WORD_SIZE - sizeof(flash_blockInfo_t)), FLASH_WORD_SIZE - chunkOffset);


		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, writeAddr + i*FLASH_WORD_SIZE, word);
		switch(status)
		{
			case HAL_OK:
			{
				// we're good
				break;
			}
			default:
			{
				// something went wrong, bail
				// this might not be the best strategy, as it leaves us in a screwed up state
				err_println("failed to write config data to flash at 0x%08X", writeAddr + i*FLASH_WORD_SIZE);
				self.writeError = true;
				return false;
			}
		}
	}

	// update counters and last entry pointer
	self.sequenceCounter = sequence;
	self.lastEntry = writeAddr + blockInfo.numWords*FLASH_WORD_SIZE;

	return true;
}

static bool _preserveJournals(void)
{
	// first figure out which page is about to be deleted
	// - there is always one free page for preservation
	// - so the next page to preserve is +2
	// as we preserve one page at a time, with a free page we can always preserve
	uintptr_t preservationPageNumber = (_blockAddrToPageNumber(self.lastEntry) + 2) % self.numPages;


	for (size_t i = 0; i < FLASH_CONFIG_MAX_PRESERVED_TYPES; ++i)
	{
		uintptr_t blockAddr = self.lastEntriesByType[i];
		if (_isValidBlockAddress(blockAddr) && ( _blockAddrToPageNumber(blockAddr) == preservationPageNumber))
		{
			flash_blockInfo_t oldInfo = *(const flash_blockInfo_t*)blockAddr;
			// copy block to new page
			if (!_writeBlock(oldInfo, blockAddr + sizeof(flash_blockInfo_t)))
			{
				// bail on write error
				return false;
			}
			// update preservation register table
			self.lastEntriesByType[i] = self.lastEntry;

		}
	}
	return true;
}

static void _deleteOldestPage(void)
{
	// it is assumed that there is always at least one more page with written data, eg. the page being deleted is not the last page
	size_t oldPage = _blockAddrToPageNumber(self.firstEntry);

	_eraseConfigPages(oldPage, 1);

	// increment entry counter
	self.firstEntry = self.configFlashStart + ((oldPage+1) % self.numPages)*FLASH_PAGE_SIZE;

}

// make room and return write address
static bool _prepareForWrite(size_t numWords)
{
	// if nothing's written, yet, we're good to start writing at the beginning
	if (self.lastEntry == 0)
		return true;

	// try to delete and preserve at most numPages/2 times to free up space for the new entry
	// why numPages/2? we don't want to end up in an infinite loop, but also don't want to be stuck because of some weird edge case while preserving the journal
	for (size_t k = 0; k < self.numPages/2; ++k)
	{
		uintptr_t endAddr = _blockEnd(self.lastEntry);

		uintptr_t pageAddr = _blockAddrToPageAddr(endAddr);
		size_t pageFreeBytes = endAddr - pageAddr;

		// if there aren't enough free bytes, we need to go to the next page, if we have enough free pages
		if (pageFreeBytes < numWords*FLASH_WORD_SIZE)
		{
			size_t newestPage = _blockAddrToPageNumber(self.lastEntry);
			size_t oldestPage = _blockAddrToPageNumber(self.firstEntry);
			size_t numFreePages = newestPage > oldestPage ? 
				self.numPages - (newestPage - oldestPage) : 
				oldestPage - newestPage;

			// if there's not enough free pages, delete one while preserving the journals
			if (numFreePages < 2)
			{
				if(!_preserveJournals())
					return false; // bail immediately if an error occurs
				_deleteOldestPage();
			}
		}
		else
		{
			break;
		}
	}

	return true;
}

// len contains buffer size, actual data copied is returned in it
// returns true, len=0 if no entry exists
// return strue, len=srclen if srclen is larger than destination buffer
bool ssf_flash_tryReadLastBlock(uint8_t flags, void* dst, size_t* len)
{
	assert(dst != NULL);
	assert(len != 0);
	assert((*len % 8) == 0); // make sure the dst buffer is sized right
	bool writeLocked = atomic_flag_test_and_set(&self.writeLock);
	if (writeLocked)
	{
		// if the lock was already taken, return false because we can't perform the write
		*len = 0;
		return false;
	}

	uint8_t type = flags & (FLASH_CONFIG_MAX_TYPES - 1u);
	uintptr_t blockAddr = self.lastEntriesByType[type];

	if (!_isValidBlockAddress(blockAddr))
	{
		*len = 0;
		atomic_flag_clear(&self.writeLock);
		return true;
	}

	const flash_blockInfo_t* blockInfo = (const void*)blockAddr;
	size_t srclen = blockInfo->numWords*FLASH_WORD_SIZE;

	if (srclen < *len)
	{
		*len = srclen;
		atomic_flag_clear(&self.writeLock);
		return true;
	}

	*len = srclen;

	memcpy(dst, (const void*)blockAddr, srclen);

	atomic_flag_clear(&self.writeLock);
	return true;
}

bool ssf_flash_tryWriteBlock(uint8_t flags, uintptr_t dataAddr, size_t datalen)
{
	bool writeLocked = atomic_flag_test_and_set(&self.writeLock);
	if (writeLocked)
	{
		dbg_println("ssf_flash_tryWriteBlock() already locked, aborting.");
		// if the lock was already taken, return false because we can't perform the write
		return false;
	}
	if (self.writeError)
	{
		// if we're in a write error state, don't try to write
		dbg_println("ssf_flash_tryWriteBlock() in write error, aborting.");
		atomic_flag_clear(&self.writeLock);
		return false;
	}
	uint16_t sequenceId = _nextSequenceNumber(self.sequenceCounter);
	size_t numWords = (sizeof(flash_blockInfo_t) + datalen + FLASH_WORD_SIZE-1)/FLASH_WORD_SIZE;

	if (!_prepareForWrite(numWords))
	{
		dbg_println("ssf_flash_tryWriteBlock() failed to prep write, aborting.");
		atomic_flag_clear(&self.writeLock);
		return false;
	}

	flash_blockInfo_t blockInfo = {
		.sequenceId = sequenceId,
		.numWords = numWords,
		.flags = flags,
	};

	if (!_writeBlock(blockInfo, dataAddr))
	{
		dbg_println("ssf_flash_tryWriteBlock() failed to write block, aborting.");
		atomic_flag_clear(&self.writeLock);
		return false;
	}

	// update last entries by type table 
	size_t type = (flags & (FLASH_CONFIG_MAX_TYPES - 1u));
	self.lastEntriesByType[type] = self.lastEntry;


	atomic_flag_clear(&self.writeLock);
	dbg_println("ssf_flash_tryWriteBlock() done.");
	return true;
}


bool ssf_flash_clearConfigFlash(void)
{
	dbg_println("ssf_flash_clearConfigFlash()...");
	bool writeLocked = atomic_flag_test_and_set(&self.writeLock);
	if (writeLocked)
	{
		// if the lock was already taken, return false because we can't perform our task
		return false;
	}

	FLASH_EraseInitTypeDef erase = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Page = (self.configFlashStart-FLASH_BASE)/FLASH_PAGE_SIZE,
		.NbPages = self.numPages,
		.Banks = FLASH_BANK_1,
	};

	dbg_println("  erasing %d pages starting at %d", erase.NbPages, erase.Page);

	_eraseConfigPages(0, self.numPages);

	// for (size_t i = 0; i < self.numPages; ++i)
	// {
	// 	FLASH_PageErase((self.configFlashStart-FLASH_BASE)/FLASH_PAGE_SIZE + i, FLASH_BANK_1);
 //        FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

 //        /* If the erase operation is completed, disable the PER Bit */
 //        CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));

	// }

	self.writeError = false;

	atomic_flag_clear(&self.writeLock);
	dbg_println("ssf_flash_clearConfigFlash() done.");
	return true;
}

bool ssf_flash_clearWriteError(void)
{
	bool writeLocked = atomic_flag_test_and_set(&self.writeLock);
	if (writeLocked)
	{
		// if the lock was already taken, return false because we can't perform our task
		return false;
	}

	if (self.writeError)
	{
		// If we do have a write error, what do we do? Erasing the flash completely will cure it, but do we have to do that?
		// FIXME: do this later, for now we'll ride assuming this cannot happen
		assert(0);
	}

	atomic_flag_clear(&self.writeLock);
	return true;
}

