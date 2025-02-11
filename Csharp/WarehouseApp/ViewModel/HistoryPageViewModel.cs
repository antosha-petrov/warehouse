using System.Collections.ObjectModel;

namespace WarehouseApp.ViewModel
{
    public class HistoryPageViewModel
    {
        public ObservableCollection<HistoryCardViewModel> Items { get; }

#pragma warning disable IDE0052
        private readonly Timer _timer;
#pragma warning restore IDE0052

        public HistoryPageViewModel()
        {
            Items = new ObservableCollection<HistoryCardViewModel>();
            _timer = new Timer(async _ => await CheckLock(), null, TimeSpan.Zero, TimeSpan.FromSeconds(2));
        }

        private async Task CheckLock()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно
            
            foreach (var item in appState.Goods!)
            {
                var matchingOrderItem = Items.FirstOrDefault(o => o.Title == item.Goods.Name);
                if (matchingOrderItem != null)
                {
                    matchingOrderItem.Shelf = item.Shelf;
                    matchingOrderItem.Cell = item.Cell;
                    matchingOrderItem.Rack = item.Rack;
                }
            }
        }

        // Асинхронный метод для загрузки элементов
        public async Task LoadItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?
                .Select(goods => new HistoryCardViewModel(goods)) ?? Enumerable.Empty<HistoryCardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        // Асинхронный метод для обновления элементов
        public async Task RefreshItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?
                .Select(goods => new HistoryCardViewModel(goods)) ?? Enumerable.Empty<HistoryCardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }
    }
}
