//! The chunk store on disk: a [`BlockStore`] over the data files of a directory, and `open`.
use super::StoreError;
use super::format::{FileKind, INDEX_FILE, StoreIndex};
use super::view::{BlockStore, Chunked};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// The largest index file that is read (a hostile file cannot make `open` allocate more).
const MAX_INDEX_BYTES: u64 = 256 << 20;

/// The data files of a store directory.
#[derive(Clone, Debug)]
pub struct DiskBlocks {
    dir: PathBuf,
}

impl DiskBlocks {
    /// The files in `dir`.
    #[must_use]
    pub fn new(dir: &Path) -> Self {
        Self { dir: dir.to_path_buf() }
    }
}

fn io_err(what: &Path, e: &std::io::Error) -> StoreError {
    StoreError::Io(format!("{}: {e}", what.display()))
}

impl BlockStore for DiskBlocks {
    fn len(&self, kind: FileKind) -> Result<u64, StoreError> {
        let path = self.dir.join(kind.file_name());
        std::fs::metadata(&path).map(|m| m.len()).map_err(|e| io_err(&path, &e))
    }

    fn read(&self, kind: FileKind, offset: u64, len: usize) -> Result<Vec<u8>, StoreError> {
        let path = self.dir.join(kind.file_name());
        let mut f = File::open(&path).map_err(|e| io_err(&path, &e))?;
        f.seek(SeekFrom::Start(offset)).map_err(|e| io_err(&path, &e))?;
        let mut buf = vec![0u8; len];
        f.read_exact(&mut buf).map_err(|e| io_err(&path, &e))?;
        Ok(buf)
    }
}

/// A chunk store on disk.
pub type ChunkStore = Chunked<DiskBlocks>;

/// Opens the store in `dir`: reads and validates the index and checks it against the data files.
/// No block is read yet; use [`Chunked::verify`] to check the contents.
pub fn open(dir: &Path) -> Result<ChunkStore, StoreError> {
    let path = dir.join(INDEX_FILE);
    let len = std::fs::metadata(&path).map_err(|e| io_err(&path, &e))?.len();
    if len > MAX_INDEX_BYTES {
        return Err(StoreError::Limit(format!(
            "{} is {len} bytes, over the limit of {MAX_INDEX_BYTES}",
            path.display()
        )));
    }
    let bytes = std::fs::read(&path).map_err(|e| io_err(&path, &e))?;
    let index = StoreIndex::decode(&bytes)?;
    Chunked::new(index, DiskBlocks::new(dir))
}
